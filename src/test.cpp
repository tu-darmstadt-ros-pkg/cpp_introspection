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

#include <introspection/introspection.h>
#include <boost/lexical_cast.hpp>

using namespace cpp_introspection;

void print_introspection(MessagePtr message, const std::string& prefix = "");

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Syntax: cpp_introspection_test <filename or path> [<message type>]" << std::endl;
    exit(1);
  }

  load(argv[1]);
  if (packages().empty()) return 1;

  if (argc < 3) {
    PackagePtr package = packages().back();
    std::cout << "Introspecting package " << package->getName() << "..." << std::endl;
    for(Package::const_iterator it = package->begin(); it != package->end(); ++it) {
      MessagePtr message = *it;
      std::cout << std::endl << "Creating an instance of " << message->getName() << "..." << std::endl;
      VoidPtr instance = message->createInstance();
      MessagePtr introspected = message->introspect(instance.get());
      print_introspection(introspected);
      std::cout << std::endl << "...and expanded ..." << std::endl;
      print_introspection(expand(introspected));
    }

  } else {
    std::cout << std::endl << "Introspecting " << argv[2] << ":" << std::endl;
    MessagePtr introspected = messageByDataType(argv[2]);
    if (!introspected) {
      std::cout << "I am sorry, I don't know that message type." << std::endl;
    } else {
      V_string fields, types;
      introspected->getFields(fields, true);
      introspected->getTypes(types, true);
      assert(fields.size() == types.size());
      for(size_t i = 0; i < fields.size(); ++i) {
        std::cout << "  " << types[i] << "\t" << fields[i] << std::endl;
      }
    }
  }

  exit(0);
}

void print_introspection(MessagePtr message, const std::string& prefix) {
  if (!message->hasInstance()) {
    std::cout << "No instance!" << std::endl;
  }

  for(Message::const_iterator it = message->begin(); it != message->end(); ++it) {
    FieldPtr field = *it;

    std::cout << prefix << std::string(field->getDataType()) << " " << std::string(field->getName()) << " = ";
    if (field->isContainer()) std::cout << "[";

    if (field->isMessage()) {
      std::cout << std::endl;
      for(std::size_t i = 0; i < field->size(); i++) {
        MessagePtr expanded = field->expand(i);
        if (!expanded) {
          std::cout << prefix << "    (unknown)" << std::endl;
          continue;
        }
        print_introspection(expanded, prefix + "    ");
      }
      std::cout << prefix;
    } else {
      for(std::size_t i = 0; i < field->size(); i++) {
        if (i > 0) std::cout << ", ";
        std::cout << field->as<std::string>(i);
      }
    }
    if (field->isContainer()) std::cout << "]";
    std::cout << std::endl;
  }
}
