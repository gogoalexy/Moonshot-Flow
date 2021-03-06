/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/ahyy/Drone/Flow/src/modules/uavcannode/dsdl/uavcan/threedr/equipment/flow/optical_flow/I2CFrameIntegral.uavcan
 */

#ifndef THREEDR_EQUIPMENT_FLOW_OPTICAL_FLOW_I2CFRAMEINTEGRAL_HPP_INCLUDED
#define THREEDR_EQUIPMENT_FLOW_OPTICAL_FLOW_I2CFRAMEINTEGRAL_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Nested Type
# Legacy I2C Integral
#

uint16 frame_count_since_last_readout
int16 pixel_flow_x_integral
int16 pixel_flow_y_integral
int16 gyro_x_rate_integral
int16 gyro_y_rate_integral
int16 gyro_z_rate_integral
uint32 integration_timespan
uint32 sonar_timestamp
uint16 ground_distance
int16 gyro_temperature
uint8 qual
******************************************************************************/

/********************* DSDL signature source definition ***********************
threedr.equipment.flow.optical_flow.I2CFrameIntegral
saturated uint16 frame_count_since_last_readout
saturated int16 pixel_flow_x_integral
saturated int16 pixel_flow_y_integral
saturated int16 gyro_x_rate_integral
saturated int16 gyro_y_rate_integral
saturated int16 gyro_z_rate_integral
saturated uint32 integration_timespan
saturated uint32 sonar_timestamp
saturated uint16 ground_distance
saturated int16 gyro_temperature
saturated uint8 qual
******************************************************************************/

#undef frame_count_since_last_readout
#undef pixel_flow_x_integral
#undef pixel_flow_y_integral
#undef gyro_x_rate_integral
#undef gyro_y_rate_integral
#undef gyro_z_rate_integral
#undef integration_timespan
#undef sonar_timestamp
#undef ground_distance
#undef gyro_temperature
#undef qual

namespace threedr
{
namespace equipment
{
namespace flow
{
namespace optical_flow
{

template <int _tmpl>
struct UAVCAN_EXPORT I2CFrameIntegral_
{
    typedef const I2CFrameIntegral_<_tmpl>& ParameterType;
    typedef I2CFrameIntegral_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > frame_count_since_last_readout;
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate > pixel_flow_x_integral;
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate > pixel_flow_y_integral;
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate > gyro_x_rate_integral;
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate > gyro_y_rate_integral;
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate > gyro_z_rate_integral;
        typedef ::uavcan::IntegerSpec< 32, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > integration_timespan;
        typedef ::uavcan::IntegerSpec< 32, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > sonar_timestamp;
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > ground_distance;
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate > gyro_temperature;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > qual;
    };

    enum
    {
        MinBitLen
            = FieldTypes::frame_count_since_last_readout::MinBitLen
            + FieldTypes::pixel_flow_x_integral::MinBitLen
            + FieldTypes::pixel_flow_y_integral::MinBitLen
            + FieldTypes::gyro_x_rate_integral::MinBitLen
            + FieldTypes::gyro_y_rate_integral::MinBitLen
            + FieldTypes::gyro_z_rate_integral::MinBitLen
            + FieldTypes::integration_timespan::MinBitLen
            + FieldTypes::sonar_timestamp::MinBitLen
            + FieldTypes::ground_distance::MinBitLen
            + FieldTypes::gyro_temperature::MinBitLen
            + FieldTypes::qual::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::frame_count_since_last_readout::MaxBitLen
            + FieldTypes::pixel_flow_x_integral::MaxBitLen
            + FieldTypes::pixel_flow_y_integral::MaxBitLen
            + FieldTypes::gyro_x_rate_integral::MaxBitLen
            + FieldTypes::gyro_y_rate_integral::MaxBitLen
            + FieldTypes::gyro_z_rate_integral::MaxBitLen
            + FieldTypes::integration_timespan::MaxBitLen
            + FieldTypes::sonar_timestamp::MaxBitLen
            + FieldTypes::ground_distance::MaxBitLen
            + FieldTypes::gyro_temperature::MaxBitLen
            + FieldTypes::qual::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::frame_count_since_last_readout >::Type frame_count_since_last_readout;
    typename ::uavcan::StorageType< typename FieldTypes::pixel_flow_x_integral >::Type pixel_flow_x_integral;
    typename ::uavcan::StorageType< typename FieldTypes::pixel_flow_y_integral >::Type pixel_flow_y_integral;
    typename ::uavcan::StorageType< typename FieldTypes::gyro_x_rate_integral >::Type gyro_x_rate_integral;
    typename ::uavcan::StorageType< typename FieldTypes::gyro_y_rate_integral >::Type gyro_y_rate_integral;
    typename ::uavcan::StorageType< typename FieldTypes::gyro_z_rate_integral >::Type gyro_z_rate_integral;
    typename ::uavcan::StorageType< typename FieldTypes::integration_timespan >::Type integration_timespan;
    typename ::uavcan::StorageType< typename FieldTypes::sonar_timestamp >::Type sonar_timestamp;
    typename ::uavcan::StorageType< typename FieldTypes::ground_distance >::Type ground_distance;
    typename ::uavcan::StorageType< typename FieldTypes::gyro_temperature >::Type gyro_temperature;
    typename ::uavcan::StorageType< typename FieldTypes::qual >::Type qual;

    I2CFrameIntegral_()
        : frame_count_since_last_readout()
        , pixel_flow_x_integral()
        , pixel_flow_y_integral()
        , gyro_x_rate_integral()
        , gyro_y_rate_integral()
        , gyro_z_rate_integral()
        , integration_timespan()
        , sonar_timestamp()
        , ground_distance()
        , gyro_temperature()
        , qual()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<200 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    // This type has no default data type ID

    static const char* getDataTypeFullName()
    {
        return "threedr.equipment.flow.optical_flow.I2CFrameIntegral";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool I2CFrameIntegral_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        frame_count_since_last_readout == rhs.frame_count_since_last_readout &&
        pixel_flow_x_integral == rhs.pixel_flow_x_integral &&
        pixel_flow_y_integral == rhs.pixel_flow_y_integral &&
        gyro_x_rate_integral == rhs.gyro_x_rate_integral &&
        gyro_y_rate_integral == rhs.gyro_y_rate_integral &&
        gyro_z_rate_integral == rhs.gyro_z_rate_integral &&
        integration_timespan == rhs.integration_timespan &&
        sonar_timestamp == rhs.sonar_timestamp &&
        ground_distance == rhs.ground_distance &&
        gyro_temperature == rhs.gyro_temperature &&
        qual == rhs.qual;
}

template <int _tmpl>
bool I2CFrameIntegral_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(frame_count_since_last_readout, rhs.frame_count_since_last_readout) &&
        ::uavcan::areClose(pixel_flow_x_integral, rhs.pixel_flow_x_integral) &&
        ::uavcan::areClose(pixel_flow_y_integral, rhs.pixel_flow_y_integral) &&
        ::uavcan::areClose(gyro_x_rate_integral, rhs.gyro_x_rate_integral) &&
        ::uavcan::areClose(gyro_y_rate_integral, rhs.gyro_y_rate_integral) &&
        ::uavcan::areClose(gyro_z_rate_integral, rhs.gyro_z_rate_integral) &&
        ::uavcan::areClose(integration_timespan, rhs.integration_timespan) &&
        ::uavcan::areClose(sonar_timestamp, rhs.sonar_timestamp) &&
        ::uavcan::areClose(ground_distance, rhs.ground_distance) &&
        ::uavcan::areClose(gyro_temperature, rhs.gyro_temperature) &&
        ::uavcan::areClose(qual, rhs.qual);
}

template <int _tmpl>
int I2CFrameIntegral_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::frame_count_since_last_readout::encode(self.frame_count_since_last_readout, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::pixel_flow_x_integral::encode(self.pixel_flow_x_integral, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::pixel_flow_y_integral::encode(self.pixel_flow_y_integral, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::gyro_x_rate_integral::encode(self.gyro_x_rate_integral, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::gyro_y_rate_integral::encode(self.gyro_y_rate_integral, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::gyro_z_rate_integral::encode(self.gyro_z_rate_integral, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::integration_timespan::encode(self.integration_timespan, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::sonar_timestamp::encode(self.sonar_timestamp, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::ground_distance::encode(self.ground_distance, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::gyro_temperature::encode(self.gyro_temperature, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::qual::encode(self.qual, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int I2CFrameIntegral_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::frame_count_since_last_readout::decode(self.frame_count_since_last_readout, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::pixel_flow_x_integral::decode(self.pixel_flow_x_integral, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::pixel_flow_y_integral::decode(self.pixel_flow_y_integral, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::gyro_x_rate_integral::decode(self.gyro_x_rate_integral, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::gyro_y_rate_integral::decode(self.gyro_y_rate_integral, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::gyro_z_rate_integral::decode(self.gyro_z_rate_integral, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::integration_timespan::decode(self.integration_timespan, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::sonar_timestamp::decode(self.sonar_timestamp, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::ground_distance::decode(self.ground_distance, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::gyro_temperature::decode(self.gyro_temperature, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::qual::decode(self.qual, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature I2CFrameIntegral_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xB477C77697571831ULL);

    FieldTypes::frame_count_since_last_readout::extendDataTypeSignature(signature);
    FieldTypes::pixel_flow_x_integral::extendDataTypeSignature(signature);
    FieldTypes::pixel_flow_y_integral::extendDataTypeSignature(signature);
    FieldTypes::gyro_x_rate_integral::extendDataTypeSignature(signature);
    FieldTypes::gyro_y_rate_integral::extendDataTypeSignature(signature);
    FieldTypes::gyro_z_rate_integral::extendDataTypeSignature(signature);
    FieldTypes::integration_timespan::extendDataTypeSignature(signature);
    FieldTypes::sonar_timestamp::extendDataTypeSignature(signature);
    FieldTypes::ground_distance::extendDataTypeSignature(signature);
    FieldTypes::gyro_temperature::extendDataTypeSignature(signature);
    FieldTypes::qual::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef I2CFrameIntegral_<0> I2CFrameIntegral;

// No default registration

} // Namespace optical_flow
} // Namespace flow
} // Namespace equipment
} // Namespace threedr

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrameIntegral >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::threedr::equipment::flow::optical_flow::I2CFrameIntegral::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrameIntegral >::stream(Stream& s, ::threedr::equipment::flow::optical_flow::I2CFrameIntegral::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "frame_count_since_last_readout: ";
    YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrameIntegral::FieldTypes::frame_count_since_last_readout >::stream(s, obj.frame_count_since_last_readout, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "pixel_flow_x_integral: ";
    YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrameIntegral::FieldTypes::pixel_flow_x_integral >::stream(s, obj.pixel_flow_x_integral, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "pixel_flow_y_integral: ";
    YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrameIntegral::FieldTypes::pixel_flow_y_integral >::stream(s, obj.pixel_flow_y_integral, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "gyro_x_rate_integral: ";
    YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrameIntegral::FieldTypes::gyro_x_rate_integral >::stream(s, obj.gyro_x_rate_integral, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "gyro_y_rate_integral: ";
    YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrameIntegral::FieldTypes::gyro_y_rate_integral >::stream(s, obj.gyro_y_rate_integral, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "gyro_z_rate_integral: ";
    YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrameIntegral::FieldTypes::gyro_z_rate_integral >::stream(s, obj.gyro_z_rate_integral, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "integration_timespan: ";
    YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrameIntegral::FieldTypes::integration_timespan >::stream(s, obj.integration_timespan, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "sonar_timestamp: ";
    YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrameIntegral::FieldTypes::sonar_timestamp >::stream(s, obj.sonar_timestamp, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "ground_distance: ";
    YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrameIntegral::FieldTypes::ground_distance >::stream(s, obj.ground_distance, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "gyro_temperature: ";
    YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrameIntegral::FieldTypes::gyro_temperature >::stream(s, obj.gyro_temperature, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "qual: ";
    YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrameIntegral::FieldTypes::qual >::stream(s, obj.qual, level + 1);
}

}

namespace threedr
{
namespace equipment
{
namespace flow
{
namespace optical_flow
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::threedr::equipment::flow::optical_flow::I2CFrameIntegral::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrameIntegral >::stream(s, obj, 0);
    return s;
}

} // Namespace optical_flow
} // Namespace flow
} // Namespace equipment
} // Namespace threedr

#endif // THREEDR_EQUIPMENT_FLOW_OPTICAL_FLOW_I2CFRAMEINTEGRAL_HPP_INCLUDED