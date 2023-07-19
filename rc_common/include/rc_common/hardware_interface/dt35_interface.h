//
// Created by jialonglong on 23-7-18.
//

#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>

namespace rc_control
{
    struct SharpIR
    {
        std::string name;
        double data;
    };

    class SharpIRHandle
    {
    public:
        SharpIRHandle()=default;
        SharpIRHandle(std::string name,SharpIR* sharp_data)
        : name_(std::move(name)),sharp_data_(sharp_data)
        {
            if (!sharp_data)
                throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                                     "'. dt35 pointer is null.");
        }

        std::string getName() const
        {
            assert(sharp_data_);
            return sharp_data_->name;
        }

        double getSharpIRdata() const
        {
            assert(sharp_data_);
            return sharp_data_->data;
        }
    private:
        std::string name_;
        SharpIR* sharp_data_;
    protected:

    };

    class SharpIRInterface
            : public hardware_interface::HardwareResourceManager<SharpIRHandle, hardware_interface::ClaimResources>
    {
    };
} // namespace rc_control
