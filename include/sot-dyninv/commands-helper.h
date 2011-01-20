/*
 * Copyright 2011, Nicolas Mansard, LAAS-CNRS
 *
 * This file is part of sot-dyninv.
 * sot-dyninv is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-dyninv is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-dyninv.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __sot_dyninv_commands_helper_H__
#define __sot_dyninv_commands_helper_H__

/* --- COMMON INCLUDE -------------------------------------------------- */
#include <dynamic-graph/command.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>


/* --- GETTER --------------------------------------------------------- */
namespace dynamicgraph {
  namespace command {



    template <class E, typename T>
      class DirectGetter
      : public Command
    {
    public:
      /// Pointer to method that sets paramter of type T
      typedef T (E::*GetterMethod) () const;

      /// Constructor
    DirectGetter(E& entity,T* ptr,
		 const std::string& docString)
      : Command(entity, std::vector<Value::Type>(), docString),
	T_ptr(ptr) {}

    protected:
      virtual Value doExecute() { return Value(*T_ptr); }
    private:
      T* T_ptr;
    };

    template <class E, typename T>
      DirectGetter<E,T>*
      makeDirectGetter( E& entity,T* ptr,
			const std::string& docString)
      { return  new DirectGetter<E,T>(entity,ptr,docString); }

    std::string docDirectGetter( const std::string& name,
				 const std::string& type )
      {
	return std::string("\nGet the ")+name+".\n\nNo input.\nReturn an "+type+".\n\n";
      }

  } // namespace command
} // namespace dynamicgraph


/* --- SETTER --------------------------------------------------------- */
namespace dynamicgraph {
  namespace command {


    template< typename T >
      struct ValueHelper
      {
	static const Value::Type TypeID;
      };

    template<>
      const Value::Type ValueHelper<int>::TypeID = Value::INT;


    template <class E, typename T, Value::Type TYPEID>
      class DirectSetter
      : public Command
    {
    public:
    DirectSetter(E& entity,T* ptr,const std::string& docString)
      :Command(entity, boost::assign::list_of(TYPEID), docString)
	,T_ptr(ptr)
      {}

    protected:
      virtual Value doExecute()
      {
	const std::vector<Value>& values = getParameterValues();
	T val = values[0].value();
	(*T_ptr) = val;
	return Value(); // void
      }
    private:
      T* T_ptr;
    };



    template <Value::Type TYPEID,class E, typename T>
      DirectSetter<E,T,TYPEID>*
      makeDirectSetter( E& entity,T* ptr,
			const std::string& docString)
      { return  new DirectSetter<E,T,TYPEID>(entity,ptr,docString); }

    std::string docDirectSetter( const std::string& name,
				 const std::string& type )
      {
	return std::string("\nSet the ")+name+".\n\nInput:\n - a "
	  +type+".\nVoid return.\n\n";
      }

  } // namespace command
} // namespace dynamicgraph



/* --- HELPER --------------------------------------------------------------- */
namespace sot {
  namespace dyninv {
    using ::dynamicgraph::command::Value;
    using ::dynamicgraph::command::makeDirectGetter;
    using ::dynamicgraph::command::docDirectGetter;
    using ::dynamicgraph::command::makeDirectSetter;
    using ::dynamicgraph::command::docDirectSetter;
  } // namespace dyninv
} // namespace sot



#endif // __sot_dyninv_commands_helper_H__


