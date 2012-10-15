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

#ifndef CPP_INTROSPECTION_MESSAGE_H
#define CPP_INTROSPECTION_MESSAGE_H

#include <introspection/forwards.h>
#include <introspection/field.h>

#include <std_msgs/Header.h>
#include <ros/time.h>

namespace cpp_introspection {

  class Message
  {
  protected:
    V_Field fields_;
    M_Field fields_by_name_;

  private:
    const Package& package_;

  public:
    Message(const Package& package) : package_(package) {}
    virtual ~Message() {}

    const Package& package() const { return package_; }

    virtual const char* getName() const = 0;
    virtual const char* getDataType() const = 0;
    virtual const char* getMD5Sum() const = 0;
    virtual const char* getDefinition() const = 0;
    virtual const std::type_info& getTypeId() const = 0;

    virtual bool isMessage() const { return true; }
    virtual bool isSimple() const = 0;
    virtual bool isFixedSize() const = 0;
    virtual bool hasHeader() const = 0;

    virtual std_msgs::Header* getHeader(const VoidPtr& instance) const = 0;
    virtual const std_msgs::Header* getHeader(const VoidConstPtr& instance) const = 0;
    virtual std::string* getFrameId(const VoidPtr& instance) const = 0;
    virtual const std::string* getFrameId(const VoidConstPtr& instance) const = 0;
    virtual ros::Time* getTimeStamp(const VoidPtr& instance) const = 0;
    virtual const ros::Time* getTimeStamp(const VoidConstPtr& instance) const = 0;

    virtual const V_Field& fields() const { return fields_; }
    virtual FieldWPtr field(const std::string& name) const { return fields_by_name_.at(name); }
    virtual V_string getFields(bool expand = false, const std::string& separator = ".") const;
    virtual V_string& getFields(V_string& fields, bool expand = false, const std::string& separator = ".", const std::string& prefix = std::string()) const;
    virtual V_string getTypes(bool expand = false) const;
    virtual V_string& getTypes(V_string& types, bool expand = false) const;

    virtual std::vector<boost::any> getValues(bool expand = false, std::size_t index = 0) const;
    virtual std::vector<boost::any>& getValues(std::vector<boost::any>& values, bool expand = false, std::size_t index = 0) const;

    virtual VoidPtr createInstance() const = 0;
    virtual void serialize(ros::serialization::OStream& stream, const VoidConstPtr& instance) const = 0;
    virtual std::size_t serializationLength(const VoidConstPtr& instance) const = 0;
    virtual VoidPtr deserialize(ros::serialization::IStream& stream, const VoidPtr& instance = VoidPtr()) const = 0;

    virtual bool hasInstance() const { return false; }
    virtual MessagePtr introspect(void *instance) const = 0;
    virtual MessagePtr introspect(void const *instance) const = 0;

    template <typename T> typename T::Ptr      narrow(const VoidPtr& instance) const      { return boost::shared_static_cast<T>(instance); }
    template <typename T> typename T::ConstPtr narrow(const VoidConstPtr& instance) const { return boost::shared_static_cast<T const>(instance); }

    typedef V_Field::iterator iterator;
    typedef V_Field::const_iterator const_iterator;
    const_iterator begin() const { return fields_.begin(); }
    const_iterator end() const   { return fields_.end(); }

  protected:
    virtual const FieldPtr& add(const FieldPtr& field);
  };

  MessagePtr messageByDataType(const std::string& data_type, const std::string& package = std::string());
  MessagePtr messageByMD5Sum(const std::string& md5sum);
  MessagePtr messageByTypeId(const std::type_info& type_info);

} // namespace cpp_introspection

#endif // CPP_INTROSPECTION_MESSAGE_H
