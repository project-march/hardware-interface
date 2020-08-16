//
// Created by roel on 16-08-20.
//

#ifndef MARCH_HARDWARE_USB_MASTER_H
#define MARCH_HARDWARE_USB_MASTER_H

#include "odrive_endpoint.h"
#include <string>
#include <vector>
#include <memory>

namespace march
{
class UsbMaster
{
public:
    UsbMaster() = default;
    std::shared_ptr<OdriveEndpoint> getSerialConnection(const std::string& serial_number);

private:
    std::vector<std::shared_ptr<OdriveEndpoint>> odrive_endpoints;


};

}  // namespace march
#endif //MARCH_HARDWARE_USB_MASTER_H
