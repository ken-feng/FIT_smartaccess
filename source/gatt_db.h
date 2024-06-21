PRIMARY_SERVICE(service_gatt, gBleSig_GenericAttributeProfile_d)
        CHARACTERISTIC(char_service_changed, gBleSig_GattServiceChanged_d, (gGattCharPropRead_c | gGattCharPropNotify_c) )
            VALUE(value_service_changed, gBleSig_GattServiceChanged_d, (gPermissionNone_c), 4, 0x00, 0x00, 0x00, 0x00)
            CCCD(cccd_service_changed)

PRIMARY_SERVICE(service_gap, gBleSig_GenericAccessProfile_d)
    CHARACTERISTIC(char_device_name, gBleSig_GapDeviceName_d, (gGattCharPropRead_c) )
            VALUE(value_device_name, gBleSig_GapDeviceName_d, (gPermissionFlagReadable_c), 12, "FIT_SMART_ACC")

/*
PRIMARY_SERVICE_UUID128(service_temperature, uuid_service_temperature)
    CHARACTERISTIC(char_temperature, gBleSig_Temperature_d, (gGattCharPropNotify_c))
        VALUE(value_temperature, gBleSig_Temperature_d, (gPermissionNone_c), 2, 0x00, 0xB4)
        DESCRIPTOR(desc_temperature, gBleSig_CharPresFormatDescriptor_d, (gPermissionFlagReadable_c), 7, 0x0E, 0xFE, 0x2F, 0x27, 0x00, 0x00, 0x00)
        CCCD(cccd_temperature)
*/
PRIMARY_SERVICE_UUID128(service_temperature, uuid_service_temperature)
    CHARACTERISTIC(char_data1, 0xBBB1, (gGattCharPropNotify_c))
        VALUE(value_data1, 0xBBB1, (gPermissionNone_c), 2, 0x00, 0xB4)
        DESCRIPTOR(desc_data1, 0xBBB2, (gPermissionFlagReadable_c), 7, 0x0E, 0xFE, 0x2F, 0x27, 0x00, 0x00, 0x00)
        CCCD(cccd_data1)
    CHARACTERISTIC(char_data2, 0xBBB3, (gGattCharPropWrite_c))
	    VALUE(value_data2, 0xBBB2, (gPermissionNone_c), 2, 0x00, 0xB4)
	    DESCRIPTOR(desc_data2, 0xBBB4, (gPermissionFlagReadable_c), 7, 0x0E, 0xFE, 0x2F, 0x27, 0x00, 0x00, 0x00)
	    CCCD(cccd_data2)
    CHARACTERISTIC(char_data3, 0xBBB5, (gGattCharPropNotify_c))
	    VALUE(value_data3, 0xBBB3, (gPermissionNone_c), 2, 0x00, 0xB4)
	    DESCRIPTOR(desc_data3, 0xBBB6, (gPermissionFlagReadable_c), 7, 0x0E, 0xFE, 0x2F, 0x27, 0x00, 0x00, 0x00)
	    CCCD(cccd_data3)
    CHARACTERISTIC(char_data4, 0xBBB7, (gGattCharPropWrite_c))
	    VALUE(value_data4, 0xBBB4, (gPermissionNone_c), 2, 0x00, 0xB4)
	    DESCRIPTOR(desc_data4, 0xBBB8, (gPermissionFlagReadable_c), 7, 0x0E, 0xFE, 0x2F, 0x27, 0x00, 0x00, 0x00)
	    CCCD(cccd_data4)

PRIMARY_SERVICE(service_battery, gBleSig_BatteryService_d)
    CHARACTERISTIC(char_battery_level, gBleSig_BatteryLevel_d, (gGattCharPropNotify_c | gGattCharPropRead_c))
        VALUE(value_battery_level, gBleSig_BatteryLevel_d, (gPermissionFlagReadable_c), 1, 0x5A)
        DESCRIPTOR(desc_bat_level, gBleSig_CharPresFormatDescriptor_d, (gPermissionFlagReadable_c), 7, 0x04, 0x00, 0xAD, 0x27, 0x01, 0x01, 0x00)
        CCCD(cccd_battery_level)

PRIMARY_SERVICE(service_device_info, gBleSig_DeviceInformationService_d)
    CHARACTERISTIC(char_manuf_name, gBleSig_ManufacturerNameString_d, (gGattCharPropRead_c) )
        VALUE(value_manuf_name, gBleSig_ManufacturerNameString_d, (gPermissionFlagReadable_c), sizeof(MANUFACTURER_NAME), MANUFACTURER_NAME)
    CHARACTERISTIC(char_model_no, gBleSig_ModelNumberString_d, (gGattCharPropRead_c) )
        VALUE(value_model_no, gBleSig_ModelNumberString_d, (gPermissionFlagReadable_c), 16, "Smart acc Demo")
    CHARACTERISTIC(char_serial_no, gBleSig_SerialNumberString_d, (gGattCharPropRead_c) )
        VALUE(value_serial_no, gBleSig_SerialNumberString_d, (gPermissionFlagReadable_c), 7, "BLESN01")
    CHARACTERISTIC(char_hw_rev, gBleSig_HardwareRevisionString_d, (gGattCharPropRead_c) )
        VALUE(value_hw_rev, gBleSig_HardwareRevisionString_d, (gPermissionFlagReadable_c), sizeof(BOARD_NAME), BOARD_NAME)
    CHARACTERISTIC(char_fw_rev, gBleSig_FirmwareRevisionString_d, (gGattCharPropRead_c) )
        VALUE(value_fw_rev, gBleSig_FirmwareRevisionString_d, (gPermissionFlagReadable_c), 5, "0.1.0")
    CHARACTERISTIC(char_sw_rev, gBleSig_SoftwareRevisionString_d, (gGattCharPropRead_c) )
        VALUE(value_sw_rev, gBleSig_SoftwareRevisionString_d, (gPermissionFlagReadable_c), 5, "0.1.0")
