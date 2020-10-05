%module CarlCore
 
%{
#include <vector>
#include "CarlCore.h"
%}

%include "std_string.i"
%include "std_vector.i"

namespace std 
{
   %template(IntVector) vector<int>;
   %template(DoubleVector) vector<double>;
   %template(StringVector) vector<string>;
   %template(ConstCharVector) vector<const char*>;
}

%rename(at) operator[];
%rename(add) operator+;
%rename(multiply) operator*;
%rename(divide) operator/;
%rename(subtract) operator-;
%rename(lessThan) operator<;
%rename(equal) operator==;
%rename(notEqual) operator!=;
%rename(incrementAdd) operator+=;
%rename(incrementSubtract) operator-=;
%rename(incrementMultiply) operator*=;
%rename(incrementDivide) operator/=;

%include "../../DeepMimic/DeepMimicCore/DeepMimicCore.h"
%include "CarlCore.h"
