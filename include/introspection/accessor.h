//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
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

#ifndef CPP_INTROSPECTION_ACCESSOR_H
#define CPP_INTROSPECTION_ACCESSOR_H

#include <introspection/message_template.h>
#include <introspection/field.h>
#include <introspection/field_traits.h>

#include <boost/type_traits.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/mpl/if.hpp>

namespace cpp_introspection {

  class AccessorBase {};

  template <typename T>
  class Accessor : public AccessorBase, public MessageTemplate<typename boost::remove_const<T>::type> {
  public:
    using MessageTemplate<typename boost::remove_const<T>::type>::MessageType;
    typedef typename boost::is_const<T>::type is_const;

  private:
    const Message& message_;
    boost::shared_ptr<T> instance_;
    V_Field fields_;
    M_Field fields_by_name_;

  public:
    Accessor(const Message& message, const boost::shared_ptr<T>& instance)
      : message_(message)
      , instance_(instance)
    {
      fields_.reserve(message.size());
      for(Message::const_iterator field = message.begin(); field != message.end(); ++field) {
        FieldPtr access = (*field)->access(*this);
        fields_.push_back(access);
        fields_by_name_[access->getName()] = access;
      }
    }
    virtual ~Accessor() {}

    bool hasInstance() const { return instance_.get(); }
    VoidPtr getInstance() const { return is_const::value ? VoidPtr() : boost::const_pointer_cast<void>(getConstInstance()); }
    VoidConstPtr getConstInstance() const { return boost::shared_static_cast<void const>(instance_); }

    T& instance() const { return *instance_; }

    PackagePtr package() const { return message_.package(); }
    const char* getPackageName() const { return message_.getPackageName(); }

    const char *getName() const { return message_.getName(); }

//    const Package& package() const { return message_.package(); }
    const V_Field& fields() const { return fields_; }
    FieldWPtr field(const std::string& name) const { return fields_by_name_.at(name); }
    const V_FieldName& getFieldNames() const { return message_.getFieldNames(); }

    void serialize(ros::serialization::OStream& stream, const VoidConstPtr&) const { message_.serialize(stream, getConstInstance()); }
    ros::SerializedMessage serialize(const VoidConstPtr&) const { return message_.serialize(getConstInstance()); }
    std::size_t serializationLength(const VoidConstPtr&) const { return message_.serializationLength(getConstInstance()); }
    VoidPtr deserialize(ros::serialization::IStream& stream, const VoidPtr&) const { return is_const::value ? VoidPtr() : message_.deserialize(stream, getInstance()); }


  private:
    template <typename FieldType>
    class FieldAccess : public FieldType {
    private:
      const Accessor& accessor_;
      mutable V_Message expanded_;
      typedef typename boost::mpl::if_<boost::is_const<T>, const typename FieldType::field_type, typename FieldType::field_type>::type field_type;
      typedef typename boost::mpl::if_<boost::is_const<T>, const typename FieldType::value_type, typename FieldType::value_type>::type value_type;

    public:
      FieldAccess(const FieldType& field, const Accessor& accessor) : FieldType(field), accessor_(accessor), expanded_(size()) {}
      virtual ~FieldAccess() {}

      const Accessor& accessor() const { return accessor_; }

      using FieldType::reference;
      field_type& reference() const { return reference(accessor().instance()); }
      value_type& reference(std::size_t i) const { return reference(accessor().instance(), i); }
      bool hasInstance() const { return true; }

      // size() should return the size of the vector in the given instance
      std::size_t size() const { return field_traits::size<typename FieldType::field_type>::value(reference()); }
      bool empty() const { return field_traits::size<typename FieldType::field_type>::empty(reference()); }
      void resize(std::size_t new_size) const { field_traits::size<typename FieldType::field_type>::resize(reference(), new_size); expanded_.resize(size()); }
      std::size_t capacity() const { return field_traits::size<typename FieldType::field_type>::capacity(reference()); }

      boost::any get(std::size_t i) const { return boost::any(reference(i)); }
      void setAny(const boost::any& value, std::size_t i) const { reference(i) = boost::any_cast<typename FieldType::value_type>(value); }

      MessagePtr expand(std::size_t i) const {
        if (i >= expanded_.size()) expanded_.resize(size());
        if (i >= expanded_.size()) return MessagePtr();
        if (expanded_[i]) return expanded_[i];

        MessagePtr message(messageByTypeId(this->getTypeId()));
        if (!message) {
          ROS_WARN_NAMED("cpp_introspection", "failed to expand field %s of type %s (unknown type)", FieldType::getName(), FieldType::getDataType());
          return MessagePtr();
        }
        expanded_[i] = message->introspect(&reference(i));
        return expanded_[i];
      }
    };

  public:
    template <class FieldType> static FieldPtr access(const FieldType& field, AccessorBase& accessor) {
      return FieldPtr(new FieldAccess<FieldType>(field, static_cast<Accessor&>(accessor)));
    }
  };

  namespace {
    struct null_deleter { void operator()(void const *) const { } };
  }

  template <typename T>
  MessagePtr MessageTemplate<T>::introspect(void *instance) const
  {
    return introspect(VoidPtr(instance, null_deleter()));
  }

  template <typename T>
  MessagePtr MessageTemplate<T>::introspect(const VoidPtr& instance) const
  {
    boost::shared_ptr<T> x(boost::shared_static_cast<T>(instance));
    if (!x) return MessagePtr();
    return MessagePtr(new Accessor<T>(*this, x));
  }

  template <typename T>
  MessagePtr MessageTemplate<T>::introspect(void const *instance) const
  {
    return introspect(VoidConstPtr(instance, null_deleter()));
  }

  template <typename T>
  MessagePtr MessageTemplate<T>::introspect(const VoidConstPtr& instance) const
  {
    boost::shared_ptr<T const> x(boost::shared_static_cast<T const>(instance));
    if (!x) return MessagePtr();
    return MessagePtr(new Accessor<T const>(*this, x));
  }

} // namespace cpp_introspection

#endif // CPP_INTROSPECTION_ACCESSOR_H
