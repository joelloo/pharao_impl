#ifndef INCLUDE_PARAMETER_LIST_H
#define INCLUDE_PARAMETER_LIST_H

#include <string>
#include <map>
#include <cstddef>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "Parameter.h"

/// \brief Implements a class that takes in parameters of types corresponding to ints, strings, float/doubles and bool.
///        The parameters are stored in a map-like data structure inside the parameter list and can be directly indexed
///        and accessed by their name, as with a map. When extracting parameters from the data structure, relevant casting
///        operations are implemented so that trying to extract a parameter with the wrong datatype throws an exception.
///
///        Inspired by and based off code from Jens Behley and FKIE by Joel.

namespace Util
{

class ParameterList
{
public:

    typedef std::map<std::string, std::shared_ptr<Parameter>> ParameterMap;

    class ConstIterator
    {
        friend class ParameterList;

    public:
        typedef Parameter value_type;
        typedef std::shared_ptr<Parameter> pointer;
        typedef Parameter& reference;
        typedef const Parameter& const_reference;
        typedef size_t size_type;
        typedef ptrdiff_t difference_type;
        typedef std::input_iterator_tag iterator_category;

        ConstIterator()
        {
        }

        ~ConstIterator()
        {

        }

        const_reference operator*() const
        {
          return *(mIterator->second);
        }

        pointer operator->() const
        {
          return (mIterator->second);
        }

        bool operator ==(const ConstIterator& other) const
        {
          return mIterator == other.mIterator;
        }

        bool operator !=(const ConstIterator& other) const
        {
          return !(other == *this);
        }

        ConstIterator& operator++()
        {
          ++mIterator;

          return *this;
        }

        const ConstIterator operator++(int)
        {
            ConstIterator it = *this;
            ++mIterator;
            return it;
        }

    private:
        explicit ConstIterator(const ParameterMap::const_iterator& it)
            : mIterator(it)
        {

        }

        ParameterMap::const_iterator mIterator;

    };

    typedef ConstIterator const_iterator;

    ParameterList();
    ~ParameterList();

    ParameterList(const ParameterList& other);
    ParameterList& operator=(const ParameterList& other);

    bool operator==(const ParameterList& other) const;
    bool operator!=(const ParameterList& other) const;

    /* \brief adds a parameter to the list and replaces an already existing parameter with the same name */
    void insert(const Parameter& param);
    /* \brief adds a parameter to the list and replaces an already existing parameter with the same name
     * Warning: the pointer is not copied. The point will be delete with the parameter list.
     **/
    void insert(std::shared_ptr<Parameter> param);

    /* \brief removes the parameter with name 'param' */
    void erase(const std::string& name);
    /* \brief is parameter with given name in the parameter list? */
    bool hasParam(const std::string& name) const;

    const Parameter& operator[](const std::string& name) const
    {
        checkParam(name);
        return *(params.find(name)->second);
    }

    /* \brief get the value of parameter with name 'name' and tries to convert it to the given type */
    template<class T>
    T getValue(const std::string& name) const
    {
        checkParam(name);
        return static_cast<T> (*params.find(name)->second);
    }

    /* \brief get the parameter with the given name */
    template<class T>
    const T* getParameter(const std::string& name) const
    {
        checkParam(name);
        T* ptr = dynamic_cast<T*> (params.find(name)->second);
        if (ptr == 0)
        {
            std::stringstream str;
            str << "Parameter with name '" << name << "' not of suitable type.";
            throw Exception(str.str());
        }

        return ptr;
    }

    template<class T>
    T* getParameter(const std::string& name)
    {
        checkParam(name);
        T* ptr = dynamic_cast<T*> (params[name]);
        if (ptr == 0)
        {
            std::stringstream str;
            str << "Parameter with name '" << name << "' not of suitable type.";
            throw Exception(str.str());
        }

        return ptr;
    }

    const_iterator begin() const;
    const_iterator end() const;

    inline size_t size() const
    {
        return params.size();
    }

    void clear();

private:
    void checkParam(const std::string& name) const;
    ParameterMap params;
};

bool parseXmlFile(const std::string& filename, ParameterList& params, const bool verbose = false);

} // namespace Util

#endif 