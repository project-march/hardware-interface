#include "march_hardware/motor_controller/odrive/odrive.h"
#include "march_hardware/motor_controller/odrive/odrive_endpoint.h"

using namespace std;

namespace march
{
OdriveEndpoint::OdriveEndpoint()
{
  if (libusb_init(&lib_usb_context_) != LIBUSB_SUCCESS)
  {
    ROS_ERROR("Odrive could not connect to endpoint USB.");
  }
}

OdriveEndpoint::~OdriveEndpoint()
{
  this->remove();

  if (lib_usb_context_ != nullptr)
  {
    libusb_exit(lib_usb_context_);
    lib_usb_context_ = nullptr;
  }
}

void OdriveEndpoint::remove()
{
  if (odrive_handle_ != nullptr)
  {
    libusb_release_interface(odrive_handle_, 2);
    libusb_close(odrive_handle_);
    odrive_handle_ = nullptr;
  }
}

int OdriveEndpoint::open_connection(const std::string& serial_number)
{
  libusb_device** usb_device_list;

  ssize_t device_count = libusb_get_device_list(lib_usb_context_, &usb_device_list);
  if (device_count <= 0)
  {
    ROS_WARN("No attached odrives found");
    return ODRIVE_COMM_ERROR;
  }

  for (size_t i = 0; i < size_t(device_count); ++i)
  {
    libusb_device* device = usb_device_list[i];
    libusb_device_descriptor desc = {};

    int result = libusb_get_device_descriptor(device, &desc);
    if (result != LIBUSB_SUCCESS)
    {
      ROS_WARN("Odrive %s error getting device descriptor", serial_number.c_str());
      continue;
    }

    if (desc.idVendor == ODRIVE_USB_VENDORID && desc.idProduct == ODRIVE_USB_PRODUCTID)
    {
      libusb_device_handle* device_handle;
      struct libusb_config_descriptor* config;

      this->attached_to_handle_ = false;
      unsigned char buf[128];

      if (libusb_open(device, &device_handle) != LIBUSB_SUCCESS)
      {
        ROS_WARN("Odrive %s error opening USB device", serial_number.c_str());
        continue;
      }

      result = libusb_get_config_descriptor(device, 0, &config);
      if (result != LIBUSB_SUCCESS)
      {
        ROS_WARN("Odrive %s error getting device descriptor", serial_number.c_str());
        continue;
      }

      int ifNumber = 2;  // config->bNumInterfaces;

      if ((libusb_kernel_driver_active(device_handle, ifNumber) != LIBUSB_SUCCESS) &&
          (libusb_detach_kernel_driver(device_handle, ifNumber) != LIBUSB_SUCCESS))
      {
        libusb_close(device_handle);

        ROS_ERROR("Odrive %s driver error", serial_number.c_str());
        continue;
      }

      if (libusb_claim_interface(device_handle, ifNumber) != LIBUSB_SUCCESS)
      {
        ROS_ERROR("Odrive %s error claiming device", serial_number.c_str());
        libusb_close(device_handle);
        continue;
      }
      else
      {
        result = libusb_get_string_descriptor_ascii(device_handle, desc.iSerialNumber, buf, 127);
        if (result <= 0)
        {
          libusb_release_interface(device_handle, ifNumber);
          libusb_close(device_handle);

          ROS_ERROR("Odrive %s error getting data", serial_number.c_str());
          continue;
        }
        else
        {
          std::stringstream stream;
          stream << uppercase << std::hex << stoull(serial_number, nullptr, 16);
          std::string sn(stream.str());

          if (sn.compare(0, strlen((const char*)buf), (const char*)buf) == 0)
          {
            ROS_INFO("Odrive with serial number; %s found", serial_number.c_str());
            this->attached_to_handle_ = true;
            this->odrive_serial_number = serial_number;

            odrive_handle_ = device_handle;
            break;
          }
        }
        if (!this->attached_to_handle_)
        {
          libusb_release_interface(device_handle, ifNumber);
          libusb_close(device_handle);
        }
      }
    }
  }

  libusb_free_device_list(usb_device_list, 1);

  return ODRIVE_COMM_SUCCESS;
}

template <typename T>
int OdriveEndpoint::getData(int id, T& value)
{
  commBuffer tx;
  commBuffer rx;
  int rx_size;

  int result = this->endpointRequest(id, rx, rx_size, tx, true, sizeof(value));
  if (result != ODRIVE_COMM_SUCCESS)
  {
    return result;
  }

  memcpy(&value, &rx[0], sizeof(value));

  return ODRIVE_COMM_SUCCESS;
}

template <typename TT>
int OdriveEndpoint::setData(int endpoint_id, const TT& value)
{
  commBuffer tx;
  commBuffer rx;
  int rx_length;

  for (int i = 0; i < int(sizeof(value)); i++)
  {
    tx.push_back(((unsigned char*)&value)[i]);
  }

  return this->endpointRequest(endpoint_id, rx, rx_length, tx, true, 0);
}

int OdriveEndpoint::execFunc(int endpoint_id)
{
  commBuffer tx;
  commBuffer rx;
  int rx_length;

  return this->endpointRequest(endpoint_id, rx, rx_length, tx, false, 0);
}

int OdriveEndpoint::endpointRequest(int endpoint_id, commBuffer& received_payload, int& received_length,
                                    const commBuffer& payload, bool ack, int length, bool read, int address)
{
  commBuffer send_buffer;
  commBuffer receive_buffer;

  unsigned char receive_bytes[ODRIVE_MAX_RESULT_LENGTH] = { 0 };

  int sent_bytes = 0;
  int received_bytes = 0;
  short received_seq_no = 0;

  this->ep_lock_.lock();

  // Prepare sequence number
  if (ack)
  {
    endpoint_id |= 0x8000;
  }
  outbound_seq_no_ = (outbound_seq_no_ + 1) & 0x7fff;
  outbound_seq_no_ |= LIBUSB_ENDPOINT_IN;
  short seq_no = outbound_seq_no_;

  // Create request packet
  commBuffer packet = createODrivePacket(seq_no, endpoint_id, length, read, address, payload);

  // Transfer packet to target
  int transfer_result =
      libusb_bulk_transfer(odrive_handle_, ODRIVE_OUT_EP, packet.data(), packet.size(), &sent_bytes, ODRIVE_TIMEOUT);
  if (transfer_result != LIBUSB_SUCCESS)
  {
    ROS_ERROR("Odrive %s error in transferring data to USB, error id %i", this->odrive_serial_number.c_str(),
              transfer_result);

    this->ep_lock_.unlock();
    return transfer_result;
  }

  else if (int(packet.size()) != sent_bytes)
  {
    ROS_ERROR("Odrive %s error not all data transferring to USB was successful", this->odrive_serial_number.c_str());
  }

  // Get response
  if (ack)
  {
    int ack_result = libusb_bulk_transfer(odrive_handle_, ODRIVE_IN_EP, receive_bytes, ODRIVE_MAX_BYTES_TO_RECEIVE,
                                          &received_bytes, ODRIVE_TIMEOUT);
    if (ack_result != LIBUSB_SUCCESS)
    {
      ROS_ERROR("Odrive %s error in acknowledging response from USB, error id %i", this->odrive_serial_number.c_str(),
                ack_result);

      this->ep_lock_.unlock();
      return ack_result;
    }

    // Push received data to buffer
    for (int i = 0; i < received_bytes; i++)
    {
      receive_buffer.push_back(receive_bytes[i]);
    }

    received_payload = decodeODrivePacket(receive_buffer, received_seq_no);
    if (received_seq_no != seq_no)
    {
      ROS_ERROR("Odrive %s error received data out of order", this->odrive_serial_number.c_str());
    }
    received_length = received_payload.size();
  }

  this->ep_lock_.unlock();
  return LIBUSB_SUCCESS;
}

std::string OdriveEndpoint::getSerialNumber()
{
  return this->odrive_serial_number;
}

void OdriveEndpoint::appendShortToCommBuffer(commBuffer& buf, short value)
{
  buf.push_back((value >> 0) & 0xFF);
  buf.push_back((value >> 8) & 0xFF);
}

void OdriveEndpoint::appendIntToCommBuffer(commBuffer& buf, const int value)
{
  buf.push_back((value >> 0) & 0xFF);
  buf.push_back((value >> 8) & 0xFF);
  buf.push_back((value >> 16) & 0xFF);
  buf.push_back((value >> 24) & 0xFF);
}

commBuffer OdriveEndpoint::decodeODrivePacket(commBuffer& buf, short& seq_no)
{
  commBuffer payload;

  memcpy(&seq_no, &buf[0], sizeof(short));
  seq_no &= 0x7fff;

  for (commBuffer::size_type i = 2; i < buf.size(); ++i)
  {
    payload.push_back(buf[i]);
  }
  return payload;
}

commBuffer OdriveEndpoint::createODrivePacket(short seq_no, int endpoint_id, short response_size, bool read,
                                              int address, const commBuffer& input)
{
  commBuffer packet;
  this->crc_ = 0;

  if ((endpoint_id & 0x7fff) == 0)
  {
    this->crc_ = ODRIVE_PROTOCOL_VERION;
  }
  else
  {
    this->crc_ = ODRIVE_DEFAULT_CRC_VALUE;
  }

  appendShortToCommBuffer(packet, seq_no);
  appendShortToCommBuffer(packet, endpoint_id);
  appendShortToCommBuffer(packet, response_size);
  if (read)
  {
    appendIntToCommBuffer(packet, address);
  }

  for (uint8_t b : input)
  {
    packet.push_back(b);
  }

  appendShortToCommBuffer(packet, this->crc_);

  return packet;
}

template int OdriveEndpoint::getData(int, bool&);
template int OdriveEndpoint::getData(int, float&);
template int OdriveEndpoint::getData(int, uint8_t&);
template int OdriveEndpoint::getData(int, uint16_t&);
template int OdriveEndpoint::getData(int, uint32_t&);
template int OdriveEndpoint::getData(int, uint64_t&);
template int OdriveEndpoint::getData(int, int8_t&);
template int OdriveEndpoint::getData(int, int16_t&);
template int OdriveEndpoint::getData(int, int32_t&);
template int OdriveEndpoint::getData(int, int64_t&);

template int OdriveEndpoint::setData(int, const bool&);
template int OdriveEndpoint::setData(int, const float&);
template int OdriveEndpoint::setData(int, const uint8_t&);
template int OdriveEndpoint::setData(int, const uint16_t&);
template int OdriveEndpoint::setData(int, const uint32_t&);
template int OdriveEndpoint::setData(int, const uint64_t&);
template int OdriveEndpoint::setData(int, const int8_t&);
template int OdriveEndpoint::setData(int, const int16_t&);
template int OdriveEndpoint::setData(int, const int32_t&);
template int OdriveEndpoint::setData(int, const int64_t&);
}  // namespace march
