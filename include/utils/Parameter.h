#ifndef INCLUDE_PARAMETER_H
#define INCLUDE_PARAMETER_H

#include <string>
#include <iostream>
#include <map>
#include <sstream>
#include <exception>

#include <boost/property_tree/ptree.hpp>

namespace Util
{
/*****************************************************************************\
 \headerfile Parameter.h
 \brief Abstract base class for parameters
 Parameters should implement the conversion
 operations, if the conversion is possible.
 
 Inspired by and based off source code from Jens Behley and FKIE by Joel.

 \*****************************************************************************/

class ParameterList;

class Parameter
{
public:
    Parameter();

    virtual ~Parameter()
    {

    }

    virtual operator bool() const
    {
      std::stringstream str;
      str << "conversion of Parameter to bool with name '" << mName
          << "' not possible.";
      throw std::exception(str.str().c_str());
    }

    virtual operator int() const
    {
      std::stringstream str;
      str << "conversion of Parameter to int with name '" << mName
          << "' not possible.";
      throw std::exception(str.str().c_str());
    }

    virtual operator unsigned int() const
    {
      std::stringstream str;
      str << "conversion of Parameter  with name '" << mName
          << "' to unsigned int not possible.";
      throw std::exception(str.str().c_str());
    }

    virtual operator double() const
    {
      std::stringstream str;
      str << "conversion of Parameter  with name '" << mName
          << "' to double not possible.";
      throw std::exception(str.str().c_str());
    }

    virtual operator float() const
    {
      std::stringstream str;
      str << "conversion of Parameter to float with name '" << mName
          << "' to double not possible.";
      throw std::exception(str.str().c_str());
    }

    virtual operator std::string() const
    {
      std::stringstream str;
      str << "conversion of Parameter to std::string with name '" << mName
          << "' not possible.";
      throw std::exception(str.str().c_str());
    }

    virtual std::string toString() const = 0;

    std::string& name()
    {
      return mName;
    }

    std::string name() const
    {
      return mName;
    }

    friend std::ostream& operator<<(std::ostream& ostr, const Parameter& param)
    {
      ostr << param.toString();
      return ostr;
    }

    friend std::ostream& operator<<(std::ostream& ostr, const Parameter* param)
    {
      ostr << param->toString();
      return ostr;
    }

    virtual std::string type() const = 0;

    // arithmetic operations.
    virtual Parameter& operator+=(const Parameter&)
    {
      std::stringstream str;
      str << "operator+ not defined for parameter of type '" << type();
      throw std::exception(str.str().c_str());
    }

    virtual bool operator<=(const Parameter&)
    {
      std::stringstream str;
      str << "operator<= not defined for parameter of type '" << type();
      throw std::exception(str.str().c_str());
    }

    /** comparison of the values. **/
    virtual bool operator==(const Parameter& other) const = 0;
    virtual bool operator!=(const Parameter& other) const
    {
      return !(*this == other);
    }

protected:
    explicit Parameter(const std::string& n) :
        mName(n)
    {
    }

    inline const std::string& getName() const { return mName; }

private:
    std::string mName;
};

class BooleanParameter: public Parameter
{
public:
    BooleanParameter(const std::string& name, const bool value);

    std::string toString() const
    {
        std::stringstream out;
        out << "<param name=\"" << getName() << "\" type=\"boolean\">" << (val ? "true" : "false") << "</param>";
        return out.str();
    }

    std::string valueStr() const
    {
      std::stringstream sstr;
      sstr << (val?"true":"false");
      return sstr.str(); /** empty string if not appropriately implemented. **/
    }

    inline bool value() const
    {
      return val;
    }

    std::string type() const
    {
      return "boolean";
    }

    operator bool() const
    {
      return val;
    }

    bool operator==(const Parameter& other) const
    {
      bool ret = false;
      if (other.type() == "boolean" && other.name() == getName())
      {
          const BooleanParameter* param = dynamic_cast<const BooleanParameter*>(&other);
          ret = (param->val == val);
      }
      return ret;
    }
private:
    bool val;
};

class IntegerParameter: public Parameter
{
public:
    IntegerParameter(const std::string& name, const int value);

    std::string toString() const
    {
        std::stringstream out;
        out << "<param name=\"" << getName() << "\" type=\"integer\">" << val << "</param>";
        return out.str();
    }

    inline int value() const { return val; }

    std::string type() const { return "integer"; }

    operator int() const
    {
      return val;
    }

    operator unsigned int() const
    {
      return static_cast<unsigned int>(val);
    }

    bool operator==(const Parameter& other) const
    {
      bool ret = false;
      if (other.type() == "integer" && other.name() == getName())
      {
          const IntegerParameter* param = dynamic_cast<const IntegerParameter*>(&other);
          ret = (param->value() == val);
      }
      return ret;
    }

    Parameter& operator+=(const Parameter& rhs)
    {
      if (rhs.type() != "integer")
      {
          std::stringstream str;
          str << "operator+= not defined for parameter of type '" << rhs.type() << "'";
          throw std::exception(str.str().c_str());
      }

      const IntegerParameter* param = dynamic_cast<const IntegerParameter*>(&rhs);
      val += param->value();

      return *this;
    }

    bool operator<=(const Parameter& rhs)
    {
      if (rhs.type() != "integer")
      {
          std::stringstream str;
          str << "operator+= not defined for parameter of type '" << rhs.type() << "'";
          throw std::exception(str.str().c_str());
      }

      const IntegerParameter* param = dynamic_cast<const IntegerParameter*>(&rhs);
      return (val <= param->value());
    }

private:
    int val;
};

class FloatParameter: public Parameter
{
public:
    FloatParameter(const std::string& name, const double value);

    std::string toString() const
    {
        std::stringstream out;
        out << "<param name=\"" << getName() << "\" type=\"float\">" << val << "</param>";
        return out.str();
    }

    inline std::string type() const
    {
      return "float";
    }

    operator float() const
    {
      return static_cast<float>(val);
    }

    operator double() const
    {
      return val;
    }

    inline double value() const
    {
      return val;
    }

    bool operator==(const Parameter& other) const
    {
        bool ret = false;
        if (other.type() == "float")
        {
            const FloatParameter* param = dynamic_cast<const FloatParameter*>(&other);
            ret = (other.name() == getName() && (std::abs(val - param->val) < 0.000000001));
        }
        return ret;
    }

    Parameter& operator+=(const Parameter& rhs)
    {
      if (rhs.type() != "float")
      {
          std::stringstream str;
          str << "operator+= not defined for parameter of type '" << rhs.type() << "'";
          throw std::exception(str.str().c_str());
      }

      const FloatParameter* param = dynamic_cast<const FloatParameter*>(&rhs);
      val += param->value();

      return *this;
    }

    bool operator<=(const Parameter& rhs)
    {
      if (rhs.type() != "float")
      {
          std::stringstream str;
          str << "operator+= not defined for parameter of type '" << rhs.type() << "'";
          throw std::exception(str.str().c_str());
      }

      const FloatParameter* param = dynamic_cast<const FloatParameter*>(&rhs);
      return val <= param->value();
    }

private:
    double val;
};

class StringParameter: public Parameter
{
public:
    StringParameter(const std::string& name, const std::string value);

    std::string toString() const
    {
        std::stringstream out;
        out << "<param name=\"" << getName() << "\" type=\"string\">" << val << "</param>";
        return out.str();
    }

    inline std::string value() const
    {
      return val;
    }

    operator std::string() const
    {
      return val;
    }

    bool operator==(const Parameter& other) const
    {
        bool ret = false;
        if (other.type() == "string" && other.name() == getName())
        {
            const StringParameter* param = dynamic_cast<const StringParameter*>(&other);
            ret = (val == param->value());
        }

        return ret;
    }

    std::string type() const
    {
      return "string";
    }

private:
    std::string val;
};

} // namespace Util

#endif /* INCLUDE_PARAMETER_H */