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

#include <introspection/field.h>
#include <introspection/type.h>
#include <introspection/introspection.h>

#include <dlfcn.h>
#include <boost/filesystem.hpp>

namespace cpp_introspection {

  static M_Package g_packages;
  static V_Package g_repository;
  static M_Message g_messages_by_name;
  static M_Message g_messages_by_md5sum;
  static M_TypeInfo_Message g_messages_by_typeid;
  static V_string g_loaded_libraries;

  PackagePtr package(const std::string& pkg)
  {
    return g_packages[pkg].lock();
  }

  const V_Package &packages() {
    return g_repository;
  }

  MessagePtr messageByDataType(const std::string& data_type, const std::string& package)
  {
    if (!package.empty()) return messageByDataType(package + "/" + data_type);
    if (data_type == "Header") return g_messages_by_name[ros::message_traits::datatype<std_msgs::Header>()].lock();
    return g_messages_by_name[data_type].lock();
  }

  MessagePtr messageByMD5Sum(const std::string& md5sum)
  {
    return g_messages_by_md5sum[md5sum].lock();
  }

  MessagePtr messageByTypeId(const std::type_info &type_info) {
    return g_messages_by_typeid[&type_info].lock();
  }

  const PackagePtr& Package::add(const PackagePtr& package)
  {
    g_repository.push_back(package);
    g_packages[package->getName()] = package;
    return package;
  }

  std::vector<std::string> Package::getMessages() const
  {
    std::vector<std::string> messages;
    for(std::vector<MessagePtr>::const_iterator it = messages_.begin(); it != messages_.end(); ++it)
      messages.push_back((*it)->getName());

    return messages;
  }

  MessagePtr Package::message(const std::string& message) const
  {
    return messageByDataType(std::string(getName()) + "/" + message);
  }

  const MessagePtr& Package::add(const MessagePtr& message)
  {
    messages_.push_back(message);
    g_messages_by_name[message->getDataType()] = message;
    g_messages_by_md5sum[message->getMD5Sum()] = message;
    g_messages_by_typeid[&(message->getTypeId())] = message;
    return message;
  }

  const FieldPtr& Message::add(const FieldPtr& field)
  {
    fields_.push_back(field);
    fields_by_name_[field->getName()] = field;
    return field;
  }

  V_string Message::getFields(bool expand, const std::string& separator) const
  {
    V_string fields;
    return getFields(fields, expand);
  }

  V_string& Message::getFields(V_string& fields, bool expand, const std::string& separator, const std::string& prefix) const
  {
    for(const_iterator it = begin(); it != end(); ++it) {
      FieldPtr field = *it;
      std::string base(prefix + field->getName());
      std::size_t index = 0;

      do {
        std::string name(base);
        if (field->isArray()) name = name + boost::lexical_cast<std::string>(index);

        if (expand && field->isMessage()) {
          MessagePtr expanded = field->expand(index);
          if (!expanded) {
            ROS_WARN("Expansion of %s failed: Could not expand field %s with unknown type %s", getDataType(), field->getName(), field->getDataType());
            continue;
          }
          expanded->getFields(fields, expand, separator, name + separator);
          continue;
        }

        fields.push_back(name);
      } while(field->isArray() && ++index < field->getSize());
    }

    return fields;
  }

  V_string Message::getTypes(bool expand) const
  {
    V_string types;
    return getTypes(types, expand);
  }

  V_string& Message::getTypes(V_string& types, bool expand) const
  {
    for(const_iterator it = begin(); it != end(); ++it) {
      FieldPtr field = *it;
      std::size_t index = 0;

      do {
        if (expand && field->isMessage()) {
          MessagePtr expanded = field->expand(index);
          if (!expanded) {
            ROS_WARN("Expansion of %s failed: Could not expand field %s with unknown type %s", getDataType(), field->getName(), field->getDataType());
            continue;
          }
          expanded->getTypes(types, expand);
          continue;
        }

        types.push_back(field->isArray() ? field->getValueType() : field->getDataType());
      } while(field->isArray() && ++index < field->getSize());
    }

    return types;
  }

  std::vector<boost::any> Message::getValues(bool expand, std::size_t index) const
  {
    std::vector<boost::any> values;
    if (!hasInstance()) return values;
    return getValues(values, expand);
  }

  std::vector<boost::any>& Message::getValues(std::vector<boost::any>& values, bool expand, std::size_t index) const
  {
    if (!hasInstance()) return values;

    for(const_iterator it = begin(); it != end(); ++it) {
      FieldPtr field = *it;
      std::size_t index = 0;

      do {
        if (expand && field->isMessage()) {
          MessagePtr expanded = field->expand(index);
          if (!expanded) {
            ROS_WARN("Expansion of %s failed: Could not expand field %s with unknown type %s", getDataType(), field->getName(), field->getDataType());
            continue;
          }
          expanded->getValues(values, expand);
          continue;
        }

        values.push_back(field->get(index));
      } while(field->isArray() && ++index < field->getSize());
    }

    return values;
  }

  MessagePtr Field::expand(std::size_t i) const {
    if (!isMessage()) return MessagePtr();
    return messageByTypeId(this->getTypeId());
  }

  using namespace boost::filesystem;
  void load(const std::string& library_or_path)
  {
    path path(library_or_path);
    if (is_directory(path)) {
      ROS_DEBUG_STREAM_NAMED(ROS_PACKAGE_NAME, "Searching directory " << path << "...");
      for(directory_iterator entry(path); entry != directory_iterator(); ++entry) {
        ROS_DEBUG_STREAM_NAMED(ROS_PACKAGE_NAME, "  " << *entry << "...");
        if (is_regular_file(entry->path())) load(entry->path().string());
      }

      return;
    }

    if (is_regular_file(path)) {
      if (path.extension() != ".so") return;
      ROS_DEBUG_STREAM_NAMED(ROS_PACKAGE_NAME, "Loading " << path << "...");

      if (std::find(g_loaded_libraries.begin(), g_loaded_libraries.end(), path.string()) != g_loaded_libraries.end()) {
        ROS_WARN_STREAM_NAMED(ROS_PACKAGE_NAME, "library " << path << " already loaded");
        return;
      }

      void *library = dlopen(path.string().c_str(), RTLD_NOW | RTLD_GLOBAL);
      const char *error = dlerror();
      if (error || !library) {
        ROS_ERROR("%s", error);
        return;
      }

      Package* (*load_fcn)() = (Package* (*)()) dlsym(library, "cpp_introspection_load_package");
      error = dlerror();
      if (error || !load_fcn) {
        ROS_WARN_NAMED(ROS_PACKAGE_NAME, "%s", error);
        dlclose(library);
        return;
      }
      Package *package __attribute__((unused)) = (*load_fcn)();

      ROS_INFO_STREAM_NAMED(ROS_PACKAGE_NAME, "Successfully loaded cpp_introspection library " << path);
      g_loaded_libraries.push_back(path.string());

//      for(Package::const_iterator it = package->begin(); it != package->end(); ++it) {
//        ROS_INFO_STREAM_NAMED(ROS_PACKAGE_NAME, "Package " << package->getName() << " contains message " << (*it)->getName() << ":");
//        V_string types = (*it)->getTypes();
//        V_string names = (*it)->getFields();
//        for(V_string::const_iterator it_type = types.begin(), it_name = names.begin(); it_type != types.end() && it_name != names.end(); ++it_type, ++it_name) {
//          ROS_INFO("  %s %s", it_type->c_str(), it_name->c_str());
//        }
//      }
    }
  }

} // namespace cpp_introspection
