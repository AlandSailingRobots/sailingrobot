Coding Guidelines
=================

Åland Sailing Robots is a project where many people are involved in the development. This makes coding guidelines a necessity. In such a project, more time is spent reading and trying to understand the code, than actually writing code. The intent of these guidelines is that the code shall look consistent throughout, and be easier to read and understand for the next person to work with it. The coding guidelines are also a way to ensure that changes in the code are made for the better. This document is intended as a support when writing, refactoring and reviewing code.

The current code is not fully compliant with these guidelines. The code written in the future will not be either. The code will never be perfect. The intent is that these guidelines shall help developers to write and change the code base so that it with time improves.


### Structure and indentation

The source code in Åland Sailing Robots is indented with tabs. Code lines shall not exceed 100 characters, assuming a tab size of 4.
Include files shall be ordered in a standard way:
In a cpp-file, the related header file

1. C-system files
2. C++-system files
3. Other libraries header files
4. Åland Sailing Robots header files

In a class, the public parts go first, then the protected and last the private. Constants and enums are declared before methods, which in turn are declared before data members.
Parts of the code that does similar things shall be located closely in a file. Class members that are closely related shall be located next to each other.

We don’t use whitespaces between function name, parenthesis and argument. When functions have several arguments, they are separated with whitespace:

```cpp
void setAngleValues(int medium, int small, int midships);
```

In functions, we put whitespace before curly brackets, between statement/ loop keywords and parentheses, and between variable names and operators:

```cpp
int SailCommand::getCommand(int relativeWind) {
    if (relativeWind < m_runningAngle) {
            return m_runningCommand;
    }
    …
}

```

Always use brackets, even for a singe-line if or loop. 

### Naming conventions

First – use meaningful names, even for variables within functions. This makes the code much easier to read and understand. One-letter names shall only be used for iteration in loops. Try to find suitable verbs for function names and descriptive nouns for data members and variables.

Avoid magic numbers. To use 45 instead of “tackingLimitAngle” will make the code less readable and it is also much more difficult to search for the instances of the number.
Avoid acronyms unless it is obvious what they represent. Common programming/ IT acronyms like DB are ok, and also domain acronyms that are well-known such as GPS.

Class data members are prefixed with m_, followed by a lowercase letter. If the name is subdivided, further parts of the name starts with uppercase letters.

```cpp
int m_beamReachAngle;
```

Constants, variables and functions are named with lowercase first, then with uppercase for each new subdivition. Classes are named with uppercase first.

```cpp
const double radiusOfEarth = 6371.0;
bool CourseCalculation::continueStarboard() {
    bool continueDirection = false;
    // check if we continue
    return continueDirection;
}
```

### Functions

Strive for small functions with one single purpose. At the very least, a function shall have one single level of abstraction.

Place a function’s variables at the narrowest scope possible, and initialize variables at declaration.

Avoid having many arguments into a function. Consider creating data structures or subdividing the function.

Use const and constexpr wherever suitable.

### Classes

Strive for small classes with one clear purpose.

Decide on special member functions. The compiler will implicitly generate:
  * A public default constructor that takes no arguments, unless a constructor is defined.
  * A public copy constructor, under certain circumstances.
  * A public copy assignment operator, under certain circumstances.
  * A public move constructor, under certain circumstances.
  * A public move assignment operator, under certain circumstances.
Make a decision if these are ok in the generated versions, add comments about it or specify them as “=default”. Disable with “=delete” or implement own versions for the class if needed.

Use the keyword “explicit” for constructors callable with one argument, to avoid unintentional type conversions.

Member functions that do not modify member data shall be const-declared.

Composition is often more appropriate than inheritance. Multiple inheritance is only allowed for implementation of several purely abstract interfaces.

### Comments

The best comments are often those that you don’t need to write. When you consider writing a comment, perhaps you can break out this part of the code into a separate function with an explanatory name? Is it sufficient to rename a variable to make the intent obvious?

```cpp
// avoid
// the GPS latitude
double m_lat;

// instead
double m_latitudeFromGPS; // no comment needed

// avoid
// keep the angle in range 0-359
if (angle >= 360){
…
}
if (angle < 0){
…
}

// instead
const int fullRevolutionDegrees = 360;
int limitAngleRangeDegrees(int angle){
    if (angle >= m_minAngle + fullRevolutionDegrees){
        return decreaseFullRevolutionAngle(angle);
    }
    if (angle < m_minAngle){
        return increaseFullRevolutionAngle(angle);
    }
    return angle;
}
```

When changing code, it is easy to forget to edit the comments, and we are left with confusing comments instead of clarifying. If there is no good way turn the comment into self-explanatory code, write the comment!

Some comments are necessary and beneficial, for example:
  * The intent of a class, what does it represent? Is it an abstract interface, a concrete implementation?
  * The reason for selected special member functions.
  * Limits on input data.
  * Error handling, such as exceptions or return error codes.
  * Transfer of ownership
  * If you needed to do a work-around for some reason, or if you don’t quite understand why that code was apparently needed to get something to work, add a comment. This helps the next person reading the code.
⋅⋅* Reference some article/ document that describes the used algorithm, or describe it briefly.

### Namespaces

Namespaces are a convenient way to avoid cluttering the default namespace. Never include entire namespaces in header files, this counteracts the point of namespaces.

```cpp
using namespace thenamespace;
```

Instead explicitly specify the needed types.

```cpp
thenamespace::thetypeorfunction
```

### Type conversions

Avoid old-style (C style) casts like

```cpp
int integerDegrees = (int) floatingPointDegrees;
```

Instead, prefer static_cast for normal type conversions

```cpp
int integerDegrees = static_cast<int>(floatingPointDegrees);
```

### Some notes on C++11/14 features

#### Smart pointers

Prefer to use the smart pointers std::unique_ptr and std::shared_ptr instead of raw pointers. Ownership is clearer in the code, and memory management is handled automatically.

If you use shared_ptr, create it from a raw pointer constructed inside the shared_ptr constructor, or use the make_shared convenience function:

```cpp
std::shared_ptr<Widget> sharedWidgetP1(new Widget);
auto sharedWidgetP2 = std::make_shared<Widget>();
```

Do not create a shared_ptr from an already existing raw pointer:

```cpp
Widget* rawWidgetP = new Widget;
std::shared_ptr<Widget> sharedWidgetP1(rawWidgetP);
// rawWidgetP can be deleted, or used to create another shared_ptr // with different reference count
```

#### auto

In C++11, a variable whose type is given as auto will be given a type that matches that of the expression used to initialize it.
Prefer auto to manifest types for variables. This has several advantages – the variables are required to be initialized, and the use of auto avoids unintentional type mismatches. It can also improve the readability of the code by removing clutter.
If code readability is significantly improved by using explicit types for variables, use them for these cases.

```cpp
// instead of
for (std::vector<Item>::iterator itemIter = items.begin();
     itemIter != items.end(); ++itemIter){
    …
}
// more readable
for (auto itemIter = items.begin();
     itemIter != items.end(); ++itemIter){
    …
}
```

#### Braced-init-lists

In C++03, braced-init-lists could be used for initializing arrays and structs with no constructor:

```cpp
struct Point { int x; int y; };
Point p = {1, 2};
```

In C++11 this has been generalized, and any object type can be created with a braced-init-list:

```cpp
// Vector takes a braced-init-list of elements.
vector<string> messages{"foo", "bar"};
```

You may use braced-init-lists. Never assign a braced-init-list to an auto-declared local variable. This may be confusing in the single-element case:

```cpp
// length is a std::initializer_list<double>
auto length = {1.23};
```

Instead use any of:

```cpp
double length = 1,23;
double length = {1,23};
auto length = 1,23;
auto length = double{1,23}
```

#### Lambda expressions

Lambda expressions are a concise way of creating anonymous function objects. They are often useful when passing functions as arguments. For example:

```cpp
std::sort(parcels.begin(), parcels.end(),
  [](Parcel firstParcel, Parcel secondParcel) {
  return firstParcel.weight() < secondParcel.weight();
});
```

Use lambda expressions where appropriate. Do not use default captures, write all lambda captures explicitly. Do not write:

```cpp
[=](int x) { return x + n; }
```

Instead use

```cpp
[n](int x) { return x + n; }
```

#### Scoped enums

Prefer scoped enums, that don’t leak names and are strongly typed. In C++98/03:

```cpp
enum SailState{
    SAIL_STATE_TACK_PORT,
    //…
};
SailState sailState = SAIL_STATE_TACK_PORT;
if (sailState < 7.3) {
    // this compiles but does not make sense
}
```

In C++11/14:

```cpp
enum class SailState{
    TACK_PORT,
    //…
};
SailState sailState = SailState::TACK_PORT;
if (sailState < 7.3) {
    // error can’t compare SailState and double
}
```

### Test code

Test code shall also follow coding guidelines. It is just as important that test code is readable, modular and maintainable as production code. However, test code does not necessarily have the same requirements regarding efficiency.

A test suite that covers the functionality of the code properly enables us to refactor without fear of introducing bugs. We can be convinced that any new bugs will be discovered in the tests. A good test suite is a foundation that brings freedom to improve the code quality. 





