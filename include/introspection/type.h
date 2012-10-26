//=================================================================================================
// Copyright (c) 2011, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef CPP_INTROSPECTION_TYPE_H
#define CPP_INTROSPECTION_TYPE_H

#include <introspection/forwards.h>

#include <boost/any.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/lexical_cast.hpp>

#include <ros/time.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#define CATCH_BAD_CAST_EXCEPTION_AND_RETURN(return_value, default_value) \
  try { \
    return return_value; \
  } catch (boost::bad_any_cast&) { \
    ROS_WARN_NAMED("cpp_introspection", "bad_any_cast exception while trying to convert a value of type %s", getName()); \
    return default_value; \
  } catch (boost::bad_lexical_cast&) { \
    return default_value; \
  } while(0)


namespace cpp_introspection {

  class Type : public boost::enable_shared_from_this<Type>
  {
  private:
    const char *name_;

  public:
    Type(const char *name) : name_(name) {}
    virtual ~Type() {}
    static const TypePtr& add(const TypePtr& type, const std::string& alias = std::string());

    virtual const char *getName() const { return name_; }
    virtual const std::type_info& getTypeId() const = 0;
    virtual bool isNumeric() const { return false; }
    virtual bool isString() const { return false; }

    template <typename T> bool is() const { return &(getTypeId()) && (getTypeId() == typeid(T)); }
    template <typename TargetType> TargetType as(const boost::any& value) const;
    template <typename SourceType> boost::any from(const SourceType& value) const;

    virtual std::string as_string(const boost::any& value) const = 0;
    virtual double      as_double(const boost::any& value) const = 0;
    virtual int         as_int(const boost::any& value) const = 0;
    virtual unsigned    as_unsigned(const boost::any& value) const = 0;
    virtual boost::any  from_string(const std::string& value) const = 0;
    virtual boost::any  from_double(double value) const = 0;
    virtual boost::any  from_int(int value) const = 0;
    virtual boost::any  from_unsigned(unsigned value) const = 0;

    struct StaticInitializer { StaticInitializer(const TypePtr& type); };
    TypePtr alias(const std::string& name) const;

    bool operator==(const Type& other) { return &getTypeId() && &other.getTypeId() && getTypeId() == other.getTypeId(); }
  };

  struct UnknownType : public Type
  {
  public:
    UnknownType() : Type("(unknown)") {}
    virtual ~UnknownType() {}
    static TypePtr Instance();

    virtual const std::type_info& getTypeId() const { return *static_cast<const std::type_info *>(0); }

    virtual std::string as_string(const boost::any& value) const    { return std::string(); }
    virtual double      as_double(const boost::any& value) const    { return std::numeric_limits<double>::quiet_NaN(); }
    virtual int         as_int(const boost::any& value) const       { return 0; }
    virtual unsigned    as_unsigned(const boost::any& value) const  { return 0; }
    virtual boost::any  from_string(const std::string& value) const { return boost::any(); }
    virtual boost::any  from_double(double value) const             { return boost::any(); }
    virtual boost::any  from_int(int value) const                   { return boost::any(); }
    virtual boost::any  from_unsigned(unsigned value) const         { return boost::any(); }
  };

  template <typename T>
  class NumericType : public Type
  {
  public:
    typedef T type;

    NumericType(const char *name) : Type(name) {}
    virtual ~NumericType() {}

    virtual const std::type_info& getTypeId() const { return typeid(type); }
    virtual bool isNumeric() const { return true; }

    virtual std::string as_string(const boost::any& value) const    { CATCH_BAD_CAST_EXCEPTION_AND_RETURN(boost::lexical_cast<std::string>(boost::any_cast<type>(value)), std::string()); }
    virtual double      as_double(const boost::any& value) const    { CATCH_BAD_CAST_EXCEPTION_AND_RETURN(double(boost::any_cast<type>(value)), std::numeric_limits<double>::quiet_NaN()); }
    virtual int         as_int(const boost::any& value) const       { CATCH_BAD_CAST_EXCEPTION_AND_RETURN(int(boost::any_cast<type>(value)), 0); }
    virtual unsigned    as_unsigned(const boost::any& value) const  { CATCH_BAD_CAST_EXCEPTION_AND_RETURN(unsigned(boost::any_cast<type>(value)), 0); }
    virtual boost::any  from_string(const std::string& value) const { return boost::any(boost::lexical_cast<type>(value)); }
    virtual boost::any  from_double(double value) const             { return boost::any(type(value)); }
    virtual boost::any  from_int(int value) const                   { return boost::any(type(value)); }
    virtual boost::any  from_unsigned(unsigned value) const         { return boost::any(type(value)); }
  };

  class BoolType : public Type
  {
  public:
    typedef std_msgs::Bool::_data_type type;

    BoolType(const char *name) : Type(name) {}
    virtual ~BoolType() {}

    virtual const std::type_info& getTypeId() const { return typeid(type); }
    virtual bool isNumeric() const { return true; }

    virtual std::string as_string(const boost::any& value) const    { CATCH_BAD_CAST_EXCEPTION_AND_RETURN(boost::lexical_cast<std::string>(boost::any_cast<type>(value)), std::string()); }
    virtual double      as_double(const boost::any& value) const    { CATCH_BAD_CAST_EXCEPTION_AND_RETURN(double(boost::any_cast<type>(value)), std::numeric_limits<double>::quiet_NaN()); }
    virtual int         as_int(const boost::any& value) const       { CATCH_BAD_CAST_EXCEPTION_AND_RETURN(int(boost::any_cast<type>(value)), 0); }
    virtual unsigned    as_unsigned(const boost::any& value) const  { CATCH_BAD_CAST_EXCEPTION_AND_RETURN(unsigned(boost::any_cast<type>(value)), 0); }
    virtual boost::any  from_string(const std::string& value) const { return boost::any(type(boost::lexical_cast<bool>(value))); }
    virtual boost::any  from_double(double value) const             { return boost::any(type(value)); }
    virtual boost::any  from_int(int value) const                   { return boost::any(type(value)); }
    virtual boost::any  from_unsigned(unsigned value) const         { return boost::any(type(value)); }
  };

  class StringType : public Type
  {
  public:
    typedef std_msgs::String::_data_type type;

    StringType(const char *name) : Type(name) {}
    virtual ~StringType() {}

    virtual const std::type_info& getTypeId() const { return typeid(type); }
    virtual bool isString() const { return true; }

    virtual std::string as_string(const boost::any& value) const    { CATCH_BAD_CAST_EXCEPTION_AND_RETURN(boost::any_cast<type>(value), std::string()); }
    virtual double      as_double(const boost::any& value) const    { CATCH_BAD_CAST_EXCEPTION_AND_RETURN(boost::lexical_cast<double>(boost::any_cast<type>(value)), std::numeric_limits<double>::quiet_NaN()); }
    virtual int         as_int(const boost::any& value) const       { CATCH_BAD_CAST_EXCEPTION_AND_RETURN(boost::lexical_cast<int>(boost::any_cast<type>(value)), 0); }
    virtual unsigned    as_unsigned(const boost::any& value) const  { CATCH_BAD_CAST_EXCEPTION_AND_RETURN(boost::lexical_cast<unsigned>(boost::any_cast<type>(value)), 0); }
    virtual boost::any  from_string(const std::string& value) const { return boost::any(type(value)); }
    virtual boost::any  from_double(double value) const             { return boost::any(boost::lexical_cast<type>(value)); }
    virtual boost::any  from_int(int value) const                   { return boost::any(boost::lexical_cast<type>(value)); }
    virtual boost::any  from_unsigned(unsigned value) const         { return boost::any(boost::lexical_cast<type>(value)); }
  };

  class TimeType : public Type
  {
  public:
    typedef ros::Time type;

    TimeType(const char *name) : Type(name) {}
    virtual ~TimeType() {}

    virtual const std::type_info& getTypeId() const { return typeid(type); }
    virtual bool isNumeric() const { return true; }

    virtual std::string as_string(const boost::any& value) const    { CATCH_BAD_CAST_EXCEPTION_AND_RETURN(boost::lexical_cast<std::string>(boost::any_cast<type>(value).toSec()), std::string()); }
    virtual double      as_double(const boost::any& value) const    { CATCH_BAD_CAST_EXCEPTION_AND_RETURN(double(boost::any_cast<type>(value).toSec()), std::numeric_limits<double>::quiet_NaN()); }
    virtual int         as_int(const boost::any& value) const       { CATCH_BAD_CAST_EXCEPTION_AND_RETURN(int(boost::any_cast<type>(value).toSec()), 0); }
    virtual unsigned    as_unsigned(const boost::any& value) const  { CATCH_BAD_CAST_EXCEPTION_AND_RETURN(unsigned(boost::any_cast<type>(value).toSec()), 0); }
    virtual boost::any  from_string(const std::string& value) const { return boost::any(type(boost::lexical_cast<double>(value))); }
    virtual boost::any  from_double(double value) const             { return boost::any(type(value)); }
    virtual boost::any  from_int(int value) const                   { return boost::any(type(double(value))); }
    virtual boost::any  from_unsigned(unsigned value) const         { return boost::any(type(double(value))); }
  };

  class DurationType : public Type
  {
  public:
    typedef ros::Duration type;

    DurationType(const char *name) : Type(name) {}
    virtual ~DurationType() {}

    virtual const std::type_info& getTypeId() const { return typeid(type); }
    virtual bool isNumeric() const { return true; }

    virtual std::string as_string(const boost::any& value) const    { CATCH_BAD_CAST_EXCEPTION_AND_RETURN(boost::lexical_cast<std::string>(boost::any_cast<type>(value).toSec()), std::string()); }
    virtual double      as_double(const boost::any& value) const    { CATCH_BAD_CAST_EXCEPTION_AND_RETURN(double(boost::any_cast<type>(value).toSec()), std::numeric_limits<double>::quiet_NaN()); }
    virtual int         as_int(const boost::any& value) const       { CATCH_BAD_CAST_EXCEPTION_AND_RETURN(int(boost::any_cast<type>(value).toSec()), 0); }
    virtual unsigned    as_unsigned(const boost::any& value) const  { CATCH_BAD_CAST_EXCEPTION_AND_RETURN(unsigned(boost::any_cast<type>(value).toSec()), 0); }
    virtual boost::any  from_string(const std::string& value) const { return boost::any(type(boost::lexical_cast<double>(value))); }
    virtual boost::any  from_double(double value) const             { return boost::any(type(value)); }
    virtual boost::any  from_int(int value) const                   { return boost::any(type(double(value))); }
    virtual boost::any  from_unsigned(unsigned value) const         { return boost::any(type(double(value))); }
  };

  TypePtr type(const std::string& name);

} // namespace cpp_introspection

#include <introspection/conversion.h>

#endif // CPP_INTROSPECTION_TYPE_H
