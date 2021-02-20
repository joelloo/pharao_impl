#include "utils/Parameter.h"

namespace Util
{

Parameter::Parameter()
{
}

BooleanParameter::BooleanParameter(const std::string& name, const bool value)
    : Parameter(name)
    , val(value)
{    
}

IntegerParameter::IntegerParameter(const std::string& name, const int value)
    : Parameter(name)
    , val(value)
{    
}

FloatParameter::FloatParameter(const std::string& name, const double value)
    : Parameter(name)
    , val(value)
{    
}

StringParameter::StringParameter(const std::string& name, const std::string value)
    : Parameter(name)
    , val(value)
{    
}

} // namespace Util