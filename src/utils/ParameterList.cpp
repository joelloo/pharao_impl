#include "utils/ParameterList.h"

namespace Util
{

ParameterList::ParameterList()
{

}

ParameterList::ParameterList(const ParameterList& other)
{
    for (ParameterMap::const_iterator it = other.params.begin(); it != other.params.end(); ++it)
    {
        params.insert(*it);
    }
}

ParameterList::~ParameterList()
{
    params.clear();
}

ParameterList& ParameterList::operator=(const ParameterList& other)
{
    if (&other != this)
    {
        params.clear();

        for (ParameterMap::const_iterator it = other.params.begin(); it != other.params.end(); ++it)
        {
            params[it->first] = it->second;
        }
    }
    return *this;
}

bool ParameterList::operator==(const ParameterList& other) const
{
    bool ret = true;
    for (ParameterMap::const_iterator it = other.params.begin(); it != other.params.end(); ++it)
    {
        const Parameter& param1 = *(it->second);
        bool found = false;
        for (ParameterMap::const_iterator it2 = params.begin(); it2 != params.end(); ++it2)
        {
            if (param1 == *(it2->second)) { found = true; }
        }

        if (!found)
        {
          ret = false;
          break;
        }
    }

    return ret;
}

bool ParameterList::operator!=(const ParameterList& other) const
{
    return !(*this == other);
}

void ParameterList::insert(const Parameter& param)
{
    ParameterMap::iterator el = params.find(param.name());
    if (el != params.end())
    {
        params.erase(el);
    }

    if (param.type() == "boolean")
    {
        const BooleanParameter* tmp = dynamic_cast<const BooleanParameter*>(&param);
        params[param.name()] = std::make_shared<BooleanParameter>(param.name(), tmp->value());
    }
    else if (param.type() == "integer")
    {
        const IntegerParameter* tmp = dynamic_cast<const IntegerParameter*>(&param);
        params[param.name()] = std::make_shared<IntegerParameter>(param.name(), tmp->value());
    }
    else if (param.type() == "float")
    {
        const FloatParameter* tmp = dynamic_cast<const FloatParameter*>(&param);
        params[param.name()] = std::make_shared<FloatParameter>(param.name(), tmp->value());
    }
    else if (param.type() == "string")
    {
        const StringParameter* tmp = dynamic_cast<const StringParameter*>(&param);
        params[param.name()] = std::make_shared<StringParameter>(param.name(), tmp->value());
    }
    else
    {
        std::stringstream str;
        str << "Bad parameter type provided to insert";
        throw std::exception(str.str().c_str());
    }
}

void ParameterList::insert(std::shared_ptr<Parameter> param)
{
    if (param != nullptr)
    {
        ParameterMap::iterator el = params.find(param->name());
        if (el != params.end())
        {
            params.erase(el);
        }
        params[param->name()] = param;
    }
}

void ParameterList::erase(const std::string& name)
{
   ParameterMap::iterator el = params.find(name);
   if (el != params.end())
   {
      params.erase(el);
   }
}

bool ParameterList::hasParam(const std::string& name) const
{
    return (params.find(name) != params.end());
}

ParameterList::const_iterator ParameterList::begin() const
{
    return ParameterList::const_iterator(params.begin());
}

ParameterList::const_iterator ParameterList::end() const
{
    return ParameterList::const_iterator(params.end());
}

void ParameterList::checkParam(const std::string& name) const
{
    if (params.find(name) == params.end())
    {
        std::stringstream str;
        str << "no parameter with name " << name << " in parameter list";
        std::cout << str.str() << std::endl;
        throw std::exception(str.str().c_str());
    }
}

void ParameterList::clear()
{
    params.clear();
}

bool parseXmlFile(const std::string& filename, ParameterList& params, const bool verbose)
{
    boost::property_tree::ptree pt;

    try
    {
        boost::property_tree::read_xml(filename, pt);
    }
    catch(...)
    {
        std::cout << "Unable to read XML from file: " << filename << std::endl;
        return false;
    }

    for (const auto& v : pt.get_child("config"))
    {
        std::string childname;
        boost::property_tree::ptree subpt;
        std::tie(childname, subpt) = v;

        if (childname != "param")
        {
            if (verbose) { std::cout << "Unrecognised child: " << childname << std::endl; }
            continue;
        }
        else
        {
            const std::string param_name = subpt.get<std::string>("<xmlattr>.name");
            const std::string param_type = subpt.get<std::string>("<xmlattr>.type");

            if (param_type == "boolean")
            {
                bool value = (subpt.data() == "true");
                auto p = std::make_shared<BooleanParameter>(param_name, value);
                params.insert(p);
            }
            else if (param_type == "integer")
            {
                int value = std::atoi(subpt.data().c_str());
                auto p = std::make_shared<IntegerParameter>(param_name, value);
                params.insert(p);
            }
            else if (param_type == "float")
            {
                double value = std::atof(subpt.data().c_str());
                auto p = std::make_shared<FloatParameter>(param_name, value);
                params.insert(p);
            }
            else if (param_type == "string")
            {
              std::string value = subpt.data();
              auto p = std::make_shared<StringParameter>(param_name, value);
              params.insert(p);
            }
            else
            {
                std::cout << "Error in xml parsing" << std::endl;
                return false;
            }
            
        }
    }

    return true;
}

} // namespace Util