# C++

## 编译器

编译器负责产生obj文件,给CPU机器码.

## 连接器

负责将obj文件连接起来.

## 变量

**变量**的实质是占用空间大小不同的数据

float,double用后缀f区分, SIZEOF()可以用来查询大小

## 函数

防止代码重复,简化程序

## 头文件

头文件的功能，复制粘贴其内容到指定位置

## 调试

调试功能，打断点，看内存

## 循环

if语句的实质，看if里面里的内容是否为0，else if 分开的

## 指针

指针的本质是存储地址的整数,int*只是让人觉得这个地址里的数据是这个类型

```c++
int main()
{
    int var = 8;
    int *ptr = &var;
    *ptr = 10;
    LOG(var);
}
```

## 引用

```C++
int main()
{
    int a = 5;
    int& ref = a;
    ref = 10;
    LOG(a);
}
```

ref不存在,只是代表了a

```c++
void Increment(int value)
{
    value++;
}

int main()
{
    int a = 5;
    int& ref = a;
    Increment(a);
    LOG(a);
}
```

不改变a的大小

```c++
void Increment(int* value)
{
    (*value)++;
}

int main()
{
    int a = 5;
    int& ref = a;
    Increment(&a);
    LOG(a);
}
```

传入了地址,修改地址里的数值

```c++
void Increment(int& value)
{
    value++;
}

int main()
{
    int a = 5;
    int& ref = a;
    Increment(a);
    LOG(a);
}
```

使用引用,简化程序,美观

## 类

可以简化代码,使代码美观,减少重复

```c++
class Player//定义一个Player类
{
public:
    int x, y;
    int speed;

    void Move(int xa, int ya)//方法
    {
       x += xa * speed;
       y += ya * speed;
    }

};

int main()
{
    Player player{};//实例化
    player.Move(1, 1);
}
```

## 类与结构体的对比

1. 类具有私有和共有性
2. 结构体可以向下兼容C语言
3. 类可以进行继承,可以有大量功能.结构体只是一种数据结构

## 简单类的例子

```c++
class Log
{
public://将不同的public功能分开
    const int LogLevelError = 0;
    const int LogLevelWarning = 1;
    const int LogLevelInfo = 2;
private:
    int m_LogLevel;//定义一个私有变量
public:
    void SetLevel(int level)
    {
        m_LogLevel = level;
    }

    void Error(const char* message)
    {
        if(m_LogLevel >= LogLevelError)
            std::cout << "[ERROR]:" << message << std::endl;
    }

    void Info(const char* message)
    {
        if(m_LogLevel >= LogLevelInfo)
            std::cout << "[INFO]:" << message << std::endl;
    }

    void Warn(const char* message)
    {
        if(m_LogLevel >= LogLevelWarning)
            std::cout << "[WARNING]:" << message << std::endl;
    }
};

int main()
{
    Log log{};
    log.SetLevel(log.LogLevelWarning);
    log.Warn("Hello!");
    log.Error("Hello!");
    log.Info("Hello!");

}
```

## 静态Static

1. 在类中定义static,不同实例初始化不会改变static的数值,可以进行共享.
2. 在函数外部,static,只在一个翻译单元中存在,不会通过链接到其他文.
3. static int i 只是函数内的数值,不会被外界所调用,生命周期长周期

## 枚举

一个数值集合,用数字来表现

```c++
enum Level
{
    num1,num2,num3
};
```

## 构造函数

作用:创建实例时,初始化该类,初始化内存和设置等

```c++
class Entity
{
public:
    float a,b;
    Entity()
    {
        a = 10;
        b = 11;
    }
}

class Entity
{
public:
    float a,b,c;
    Entity() : a(11), b(12), c(10)
    {
    }
};
```

## 析构函数

一个对象被销毁时使用,可以用来释放和删除类中不想要的

```c++
class Entity
{
public:
    Entity()//构造函数,初始化类
    {
    }
    ~Entity()//析构函数,超过函数作用域生效
    {
    }
}
```

## 继承

```c++
class Entity//创建一个基类
{
public:
    float X, Y;

    void Move(float xa, float ya)
    {
        X += xa;
        Y += ya;
    }
};

class Player : public Entity//继承,集合大于基类,拥有基类的功能
{
public:
    const char* Name;

    void PrintName()
    {
        std::cout << Name << std::endl;
    }
};
```

## 虚函数

用于覆写某些程序

```c++
class Entity
{
public:
   virtual std::string GetName(){return "Entity";}
};

class Player : public Entity
{
private:
    std::string m_Name;
public:
    Player(const std::string& name)
        : m_Name(name){}

    std::string GetName() override {return  m_Name;} //重写GetName方法
};

void PrintName(Entity* entity)
{
    std::cout << entity->GetName() << std::endl;
}

int main()
{
    Entity* e = new Entity();
    PrintName(e);

    Player* p = new Player("Cherno");
    PrintName(p);
}
```

## 纯虚函数(接口)

允许在基类定义一个没有实现的函数,然后强制子类去实现该函数

```c++
class Entity
{
public:
   virtual std::string GetName() = 0;
};

class Player : public Entity
{
private:
    std::string m_Name;
public:
    Player(const std::string& name)
        : m_Name(name){}

    std::string GetName() override {return  m_Name;}
};
virtual std::string GetName() = 0;
```

纯虚函数必须被实现,才能创建这个类的实例.

```c++
class Printable
{
public:
    virtual std::string GetClassName() = 0;//纯虚数.可以作为接口
};

class Entity : public Printable
{
public:
   virtual std::string GetName(){return "Entity";}
   std::string GetClassName(){return "Entity";}
};

class Player : public Entity
{
private:
    std::string m_Name;
public:
    Player(const std::string& name)
        : m_Name(name){}

    std::string GetName() override {return  m_Name;}
    std::string GetClassName() override {return "Player";}
};

void PrintName(Entity* entity)
{
    std::cout << entity->GetName() << std::endl;
}

void Print(Printable* obj)
{
    std::cout << obj->GetClassName() << std::endl;
}

int main()
{
    Entity* e = new Entity();
//    PrintName(e);

    Player* p = new Player("Cherno");
//    PrintName(p);
    Print(e);
    Print(p);
}

```

## 可见性

- **private**:只有当前类可以访问这些变量,不能被外界访问
- **protect**:当前类和子类都可以访问,类外不可以访问
- **public**:所有人都可以访问

基于可见性,可以找到想要的访问和利用类的方式,可以便于理解

## 数组

```c++
int example[5];
int* ptr = example;

for(int i = 0; i < 5; i++)
    example[i] = 2;

example[2] = 5;
*(ptr + 2) = 6;//按照类型进行地址偏移
```

```c++
int example[5];//栈上创建,它的生存期与包含它的函数相关联.当函数执行完毕时，这个数组将自动销毁，不需要手动释放内存。
int* another = new int[5];//堆上创建,这个数组的生存期不受限制，直到你使用 delete[] 操作符来释放内存为止。如果不释放，将会导致内存泄漏。
delete[] another; // 释放动态分配的整数数组内存
```

## 字符串

```c++
char* name = "Cherno";
char name2[7] = {'C','h', 'e', 'r', 'n', 'o','\0'};//可以用数组来表示,末尾时空终止字符
```

```c++
 std::string name = "Cherno";//创建一个string类,生成一个字符串
```



## 字符串字面量

在C++中，字符串字面量是一种特殊的字符串，通常用于表示文本常量。字符串字面量是由双引号括起来的字符序列，可以直接在C++代码中使用。例如：

```c++
cppCopy code
const char* str = "Hello, World!";
```

在上面的示例中，`"Hello, World!"` 就是一个字符串字面量。注意以下几点关于字符串字面量的特性：

1. **以空字符结尾**：C++字符串字面量以空字符 `'\0'` 结尾，用于表示字符串的结束。这个空字符在内存中占用一个字符的位置。

2. **类型**：字符串字面量的类型是 `const char*`，即一个指向字符的常量指针。这是因为字符串字面量通常是不可修改的。

3. **不可修改**：字符串字面量是常量，因此不能通过指向它的指针来修改其中的字符。如果尝试这样做，会导致未定义行为。

4. **自动分配内存**：字符串字面量会在程序的静态存储区（static storage area）中分配内存，因此不需要显式地分配或释放内存。这也意味着字符串字面量在程序的整个生命周期内都可用。

5. **特殊字符转义**：字符串字面量可以包含特殊字符转义序列，如 `\n` 表示换行，`\t` 表示制表符等。

6. **多行字符串字面量（C++11及以后版本）**：在C++11及以后的标准中，可以使用多行字符串字面量来表示跨多行的字符串文本，例如：

   ```c++
   cppCopy codeconst char* multilineStr = R"(
       This is a
       multi-line
       string literal
   )";
   ```

   多行字符串字面量以 `R"(` 开始，以 `)"` 结束，中间的内容会被作为字符串原样保留。

字符串字面量在C++中广泛用于表示常量字符串，如错误消息、文本消息、文件路径等等。需要注意的是，如果要在程序中对字符串进行修改，应该使用字符数组或`std::string`等可变字符串类型，而不是字符串字面量。

## const

**const**:承诺某些量为恒定值,一般来说只读不取

```c++
const int* a = new int;//不能修改该指针指向的内容
int* const a = new int;//不能修改指针指向的地址
```

```c++
class Entity
{
private:
    int m_X, m_Y;
public:
    int GetX() const //方法不会修改实际的类
    {
        return m_X;
    }
};
```

```c++
void PrintEntity(const Entity& e)//该函数只能使用const方法
{
    std::cout << e.GetX() << std::endl;
}
```

## mutable

在**const**类中使用

```c++
mutable int var; //可以在const中修改变量
```

## 成员初始化列表

```c++
class Example
{
public:
    Example()
    {
        std::cout << "Created Entity!!!" << std::endl;
    }

    Example(int x)
    {
        std::cout << "Created Entity with " << x << std::endl;
    }
};

class Entity
{
private:
    std::string m_Name;
    Example m_Example;// 会初始化一次
public:
    Entity()
    {
        m_Name = std::string("UnKnow");
        m_Example = Example(8);//会初始化二次
    }
    Entity(const std::string& name)
    {
        m_Name = name;
    }

    const std::string& GetName() const {return m_Name;}
};
```

```c++
class Entity
{
private:
    std::string m_Name;
    Example m_Example;
public:
    Entity()
        : m_Example(8)// 成员初始化列表
    {
        m_Name = std::string("UnKnow");
    }
    Entity(const std::string& name)
    {
        m_Name = name;
    }

    const std::string& GetName() const {return m_Name;}
};
```

## 三元操作符

```c++
if (s_Level>10):
	rank = "Master";
else:
	rank = "Beginner"

std::string rank = s_Level > 10 ? "Master" : "Beginner"    
```

优化代码,使代码简单,不建议使用**三元操作符嵌套**

## 创建并初始化C++对象

```c++
Entity* e;
{
    Entity entity("Jinhemu");
    e = &entity;
    std::cout << entity.GetName() << std::endl;
}//创建一个栈分配,更快,自动释放entity
```

```c++
Entity* entity = new Entity("Jinhemu");//创建一个堆分配,比栈分配慢
delete entity;//释放对象
```

## new

在C++中，`new`是一个运算符，用于在动态内存（堆上）分配内存。它的主要作用是为对象分配内存并返回一个指向该内存的指针。使用`new`分配的内存需要使用`delete`运算符来释放。

```c++
int* b = new int[50];
Entity* e = new Entity();
delete e;
delete[] b;
```

几点需要注意的地方：

1. 使用`new`时，如果内存不足以分配所请求的对象，它会抛出一个`std::bad_alloc`异常。如果你不希望它抛出异常，而是返回一个空指针，你可以使用`new(std::nothrow)`。
2. `new`不仅仅是分配内存。它还会调用对象的构造函数。相应地，`delete`不仅释放内存，还会调用对象的析构函数。
3. 要确保为使用`new`分配的每块内存调用对应的`delete`或`delete[]`。否则，你可能会遇到内存泄漏问题。
4. 当分配数组时，使用`new[]`，并与`delete[]`一起使用，以确保正确地释放内存并调用每个对象的析构函数。
5. 现代C++编程中，直接使用裸指针（raw pointers）和`new/delete`是不鼓励的。建议使用智能指针（如`std::unique_ptr`和`std::shared_ptr`）来自动管理内存，从而减少内存泄漏和其他相关问题的风险。

## 隐式转换与explicit

在C++中，隐式转换（或称为自动转换）是指编译器在没有明确指示的情况下自动进行的类型转换。这样的转换主要是为了使表达式中的操作数类型相匹配，或满足函数调用中参数的类型要求。

隐式转换可以发生在以下几种情况：

1. **算术转换**：当两个不同的基本数据类型参与算术运算时，较小的类型可能会被转换为较大的类型。例如：

   ```c++
   cppCopy codedouble d = 5.5;
   int i = 2;
   double result = d + i;  // 这里，i会被隐式转换为double
   ```

2. **从派生类到基类的转换**：派生类的对象可以隐式地转换为基类的引用或指针。

   ```c++
   cppCopy codeclass Base {};
   class Derived : public Base {};
   
   Derived d;
   Base& b = d;  // 这里，Derived对象d被隐式转换为Base的引用
   ```

3. **转换为`bool`**：在条件表达式中，非bool类型的值会被转换为bool。例如，指针或整数值`0`将被转换为`false`，非零值将被转换为`true`。

4. **构造函数和转换运算符**：如果某个类定义了单参数构造函数或转换运算符，那么在适当的情况下，它们可能会导致隐式转换。

   ```c++
   cppCopy codeclass MyString {
   public:
       MyString(const char* str) { /*...*/ }
   };
   
   MyString s = "hello";  // 这里，const char*被隐式转换为MyString
   ```

5. **数组到指针的转换**：数组名称可以被隐式地转换为指向其首元素的指针。

   ```c++
   cppCopy codeint arr[10];
   int* p = arr;  // 这里，arr被隐式转换为指向其首元素的指针
   ```

尽管隐式转换在很多情况下都是有用的，但它也可能导致意外的行为或性能问题。因此，建议开发者时刻留意并在必要时使用显式转换（例如：使用C++的`static_cast`）来明确转换的意图。

另外，使用编译器的警告选项可以帮助检测到某些可能导致问题的隐式转换，这样你就可以对其进行审查或修改。

**explicit**:用于指示编译器不要隐式地执行构造函数的类型转换.

## 运算符和重载运算符

重载运算符是一种C++的特性，允许您自定义如何使用内置运算符或其他自定义运算符来执行操作。通过重载运算符，您可以定义类对象之间的操作行为，使其更符合您的需求。

```c++
struct Vector2
{
    float x, y;
    Vector2(float x, float y)
        : x(x), y(y) {}

    Vector2 Add(const Vector2& other) const
    {
        return Vector2(x + other.x, y + other.y);
    }
    Vector2 operator+(const Vector2& other) const
    {
        return Add(other);
    }
};
int main()
{
    Vector2 position(4.0f, 4.0f);
    Vector2 speed(0.5f,1.5f);

    Vector2 result1 = position.Add(position);
    Vector2 result2 = position + speed;
}
```

## this

在C++中，`this` 是一个特殊的指针，它用于表示当前对象的地址。`this` 指针是一个成员函数的隐式参数，它始终指向调用成员函数的对象。通过使用 `this` 指针，您可以在成员函数内访问对象的成员变量和成员函数。

以下是关于 `this` 指针的一些重要事项：

1. **用法**：在成员函数内部，可以使用 `this` 指针来访问对象的成员变量和成员函数，以区分局部变量和成员变量，或者在成员函数中访问其他成员函数。

   ```cpp
   class MyClass {
   public:
       int data;

       void SetData(int value) {
           this->data = value; // 使用 this 指针访问成员变量
       }

       void PrintData() {
           std::cout << "Data: " << this->data << std::endl; // 使用 this 指针访问成员变量
       }
   };
   ```

2. **隐式使用**：通常情况下，您无需显式使用 `this` 指针，因为它在成员函数内是隐式的。例如，您可以直接访问成员变量和成员函数，而不必使用 `this` 指针：

   ```cpp
   class MyClass {
   public:
       int data;

       void SetData(int value) {
           data = value; // 不使用 this 指针，也是有效的
       }

       void PrintData() {
           std::cout << "Data: " << data << std::endl; // 不使用 this 指针，也是有效的
       }
   };
   ```

3. **用于解决命名冲突**：在一些情况下，使用 `this` 指针可以帮助解决命名冲突。如果局部变量和成员变量具有相同的名称，可以使用 `this` 指针来明确指定访问的是成员变量。

   ```cpp
   class MyClass {
   public:
       int data;
   
       void SetData(int data) {
           this->data = data; // 使用 this 指针来指定成员变量
       }
   };
   ```

总之，`this` 指针在C++中用于访问当前对象的成员。它可以帮助区分局部变量和成员变量，尤其在命名冲突的情况下很有用。然而，在大多数情况下，它是隐式的，您不需要显式地使用它来访问对象的成员。

## C++对象生存区

```c++
class example
{
public:
    example()
    {
        std::cout << "create a class" << std::endl;
    }
    ~example()
    {
        std::cout << "create a class" << std::endl;
    }
};
```

```c++
{
    example a;//生存期只在这个区间内
}
```

```c++
{
    example* a = new example;//生存期直到释放或者是程序结束
}
```

```c++
int* CreateArray()
{
    int array[50];
    return array;//一个指向栈内存的指针
}
```

对象的生存期是指对象存在的时间段，即从对象创建到销毁的时间段。在C++中，对象的生存期受到其创建方式、作用域和存储类别的影响。

以下是关于对象生存期的一些重要概念：

1. **自动生存期**：自动生存期对象是在局部作用域内创建的对象，它们在程序执行到达声明它们的作用域结束时被销毁。这些对象的生命周期受到大括号 `{}` 的界定。例如：

   ```cpp
   void myFunction() {
       int x = 10; // x 是自动生存期对象
   } // x 超出作用域，被销毁
   ```

2. **静态生存期**：静态生存期对象是在程序启动时创建的，直到程序结束才被销毁。它们通常由 `static` 关键字声明。例如：

   ```cpp
   static int count = 0; // count 是静态生存期对象
   ```

3. **动态生存期**：动态生存期对象是通过动态内存分配创建的，它们的生命周期由程序员显式控制。这种对象使用 `new` 运算符创建，在不再需要时必须使用 `delete` 运算符销毁，否则会导致内存泄漏。例如：

   ```cpp
   int* ptr = new int; // 动态分配的对象，需要手动释放
   delete ptr; // 销毁对象，释放内存
   ```

对象的生存期非常重要，因为它决定了对象何时被创建和销毁，从而影响到对象的可用性和资源管理。了解和正确管理对象的生存期对于编写高效和健壮的C++程序非常重要。不正确地管理对象的生存期可能导致内存泄漏或访问已销毁对象的问题。因此，程序员应该仔细考虑对象的生存期，并根据需要采用适当的创建和销毁策略。

## 智能指针

智能指针是C++中的一种重要工具，用于管理动态分配的内存，以减少内存泄漏和资源管理错误。智能指针是标准C++库提供的类，它们封装了指针，提供了自动内存管理功能，包括对象的自动销毁。主要的智能指针类型有以下两种：

1. **`std::shared_ptr`**：共享指针
   - `std::shared_ptr` 允许多个智能指针共享同一个堆内存对象，当最后一个引用计数为零时，堆内存会被自动释放。
   - 它使用引用计数来跟踪对象的所有者数量，确保在不再需要时正确释放内存。

   ```cpp
   #include <memory>
   
   std::shared_ptr<int> sharedPtr = std::make_shared<int>(42);
   ```

2. **`std::unique_ptr`**：独占指针
   - `std::unique_ptr` 表示对堆内存对象的唯一所有权，当 `std::unique_ptr` 超出范围时，它自动销毁关联的对象。
   - 它不使用引用计数，而是使用独占所有权来确保资源的安全管理。

   ```cpp
   #include <memory>
   
   std::unique_ptr<int> uniquePtr = std::make_unique<int>(42);
   ```

使用智能指针可以避免内存泄漏和悬挂指针（dangling pointers）等与手动内存管理相关的问题。智能指针的生命周期管理是自动的，因此可以显著减少程序中的资源管理错误。

除了上述两种主要类型外，C++标准库还提供了其他智能指针，如 `std::weak_ptr` 用于打破 `std::shared_ptr` 的循环引用。使用智能指针可以大大提高代码的可维护性和安全性，减少了手动管理内存的复杂性。

下面是一个使用 `std::shared_ptr` 的示例：

```cpp
#include <memory>
#include <iostream>

int main() {
    std::shared_ptr<int> sharedPtr = std::make_shared<int>(42);
    std::cout << "Value: " << *sharedPtr << std::endl;
    return 0; // sharedPtr 在main函数结束后自动释放内存
}
```

在这个示例中，`sharedPtr` 持有一个 `int` 类型的共享指针，当程序离开 `main` 函数时，`sharedPtr` 会自动释放分配的内存。这允许您安全地使用动态分配的资源而无需手动释放它们。

## 复制和拷贝构造函数(不懂)

在C++中，复制和拷贝构造函数是用于创建一个对象的副本的两种方式。它们通常用于类的对象，以便在不修改原始对象的情况下创建新对象，或者在对象之间传递值。

1. 复制构造函数（Copy Constructor）:
复制构造函数是一种特殊的构造函数，用于创建一个新对象，该新对象是已存在对象的副本。复制构造函数的原型如下：

```cpp
ClassName(const ClassName &other);
```

其中，`ClassName` 是类的名称，`other` 是要复制的已存在对象。复制构造函数通常以引用方式接受参数，以避免不必要的对象复制，提高效率。复制构造函数会复制已存在对象的成员变量到新对象，以创建一个一模一样的新对象。

示例：

```cpp
class MyString {
public:
    MyString(const MyString &other);  // 复制构造函数
    // 其他成员和方法
};
```

2. 拷贝构造函数（Copy Assignment Operator）:
拷贝构造函数用于将一个已存在对象的值赋给另一个已存在对象，而不是创建一个新对象。拷贝构造函数的原型如下：

```cpp
ClassName &operator=(const ClassName &other);
```

其中，`ClassName` 是类的名称，`other` 是要从中复制值的已存在对象。拷贝构造函数通常返回一个引用，以允许多次赋值操作，以及链式赋值操作。

示例：

```cpp
class MyString {
public:
    MyString &operator=(const MyString &other);  // 拷贝构造函数
    // 其他成员和方法
};
```

请注意，拷贝构造函数和复制构造函数在名称和工作方式上有所不同，因此要根据需要选择适当的函数来创建新对象或将值复制到已存在对象。通常情况下，编译器会自动生成默认的复制构造函数和拷贝构造函数，但如果类中包含指针或资源管理，你可能需要手动编写它们以确保正确的复制和赋值行为。

## 箭头操作符

在C++中，箭头操作符 `->` 通常用于访问类的成员函数和成员变量，当操作对象是指向类对象的指针时特别有用。箭头操作符的主要用途是通过指针间接访问成员。

通常，当你有一个指向类对象的指针，并想要调用该对象的成员函数或访问其成员变量时，你可以使用箭头操作符 `->`。

下面是一些示例：

1. 访问成员变量：

```cpp
class MyClass {
public:
    int value;
};

MyClass obj;
obj.value = 42;  // 直接访问对象的成员变量

MyClass* ptr = &obj;
ptr->value = 24; // 使用箭头操作符通过指针访问对象的成员变量
```

2. 调用成员函数：

```cpp
class MyClass {
public:
    void print() {
        std::cout << "Hello, World!" << std::endl;
    }
};

MyClass obj;
obj.print();  // 直接调用对象的成员函数

MyClass* ptr = &obj;
ptr->print(); // 使用箭头操作符通过指针调用对象的成员函数
```

箭头操作符使你能够以类似于直接访问对象的方式来操作对象的成员，但通过指针进行间接操作。这对于处理动态分配的对象或操作类的对象数组时非常有用。

需要注意的是，箭头操作符 `->` 仅适用于指向类对象的指针。如果你有一个类对象而不是指针，应该使用点操作符 `.` 来访问成员。

## 动态数组

动态数组是一种在程序运行时动态分配内存的数据结构，允许你在不提前知道数组大小的情况下存储和管理数据。C++中，你可以使用标准库容器（例如`std::vector`）来创建动态数组，也可以使用动态分配的静态数组（使用`new`和`delete`关键字）。

下面是两种创建动态数组的方法：

1. 使用 `std::vector`（标准库容器）：

```cpp
#include <vector>

std::vector<int> dynamicArray; // 创建一个动态整数数组

dynamicArray.push_back(10);    // 向数组中添加元素
dynamicArray.push_back(20);
dynamicArray.push_back(30);

// 访问数组元素
int value = dynamicArray[1]; // 获取索引为1的元素，值为20
```

`std::vector`会自动管理内存，可以自动扩展和收缩数组，不需要手动分配和释放内存。

2. 使用动态分配的静态数组：

```cpp
int* dynamicArray = new int[3]; // 创建一个具有3个整数的动态数组

dynamicArray[0] = 10; // 设置数组元素的值
dynamicArray[1] = 20;
dynamicArray[2] = 30;

// 访问数组元素
int value = dynamicArray[1]; // 获取索引为1的元素，值为20

// 记得释放内存，以防止内存泄漏
delete[] dynamicArray;
```

使用 `new` 和 `delete` 分配和释放内存时，务必小心，确保在不再需要数组时释放内存，以防止内存泄漏。

通常情况下，使用 `std::vector` 更安全和方便，因为它自动处理内存管理，而不需要手动释放内存。只有在需要更多控制时才使用动态分配的静态数组。

## stdvector的优化问题

`std::vector` 是 C++ 标准库中的动态数组容器，通常提供了高效的随机访问和动态大小调整的功能。然而，为了充分利用 `std::vector` 的性能，并避免不必要的开销，你可以考虑以下一些优化建议：

1. 预分配容量：`std::vector` 动态增长时会分配新的内存块，这可能导致重新分配和复制元素的开销。如果你知道 `std::vector` 预计会存储大量元素，可以使用 `reserve` 函数来预分配足够的容量，以减少重新分配的次数。

   ```cpp
   std::vector<int> myVector;
   myVector.reserve(1000); // 预分配1000个元素的容量
   ```

2. 使用移动语义：C++11 引入了移动语义，允许将资源从一个对象转移到另一个对象，而不必进行深层复制。当你需要将一个 `std::vector` 转移到另一个时，可以使用移动语义来提高性能。

   ```cpp
   std::vector<int> sourceVector;
   // 填充 sourceVector
   std::vector<int> targetVector = std::move(sourceVector); // 移动 sourceVector 到 targetVector
   ```

3. 避免频繁插入和删除：频繁的插入和删除操作可能导致 `std::vector` 多次重新分配内存和复制元素，影响性能。如果需要频繁插入和删除操作，考虑使用 `std::deque` 或 `std::list` 等数据结构，它们对插入和删除操作更高效。

4. 使用范围迭代器：C++11 引入了范围迭代器，可以通过范围 for 循环更容易地迭代 `std::vector` 中的元素。这通常比传统的索引迭代更安全和方便。

   ```cpp
   for (const int &element : myVector) {
       // 访问 element
   }
   ```

5. 使用成员函数而不是 STL 算法：`std::vector` 提供了一些成员函数，如 `push_back`、`pop_back`、`emplace_back` 等，它们在特定情况下可能比使用标准库算法更高效。

6. 使用合适的数据类型：选择合适的数据类型，以减少内存占用和提高性能。如果只需要存储整数，不要使用 `std::vector<double>`，而是使用 `std::vector<int>`。

7. 小心避免无效的迭代器：当插入或删除元素后，迭代器可能会变得无效。确保在使用迭代器时不会引发未定义行为。

综而言之，优化 `std::vector` 的性能通常涉及到合理的内存管理、使用移动语义、避免不必要的复制和重新分配，以及根据特定需求选择合适的容器类型。最佳优化策略通常取决于具体的应用场景。

`emplace_back` 是 C++ 标准库中 `std::vector` 类的成员函数之一，用于在动态数组的末尾添加新元素。与 `push_back` 不同，`emplace_back` 允许你通过参数传递给元素类型的构造函数，直接在 `std::vector` 中构造新元素，而不需要创建一个临时对象。

以下是 `emplace_back` 的用法示例：

```cpp
#include <vector>

struct MyClass {
    int x;
    double y;

    MyClass(int a, double b) : x(a), y(b) {}
};

int main() {
    std::vector<MyClass> myVector;

    // 使用 emplace_back 直接在 vector 中构造新元素
    myVector.emplace_back(42, 3.14);
    myVector.emplace_back(10, 2.71);

    // myVector 现在包含两个 MyClass 对象，而不是两个临时对象

    return 0;
}
```

在上述示例中，`emplace_back` 接受构造函数的参数，并直接在 `std::vector` 中构造新的 `MyClass` 对象。这可以避免创建临时对象并进行不必要的复制，从而提高性能。

`emplace_back` 对于避免不必要的对象复制和移动非常有用，特别是当元素类型的构造函数包含多个参数时。它允许你以更直接的方式向 `std::vector` 添加元素，而不需要手动创建临时对象，然后再将它们添加到容器中。

## 处理多个返回值

在 C++ 中，处理多个返回值有几种常见的方法。这些方法包括：

1. **使用结构体或类**：你可以创建一个自定义的结构体或类，将多个值打包到一个对象中，然后从函数中返回该对象。这种方法非常灵活，因为你可以定义包含任意数量的数据成员的结构体或类。

   ```cpp
   struct Result {
       int value1;
       double value2;
   };

   Result myFunction() {
       Result result;
       result.value1 = 42;
       result.value2 = 3.14;
       return result;
   }

   int main() {
       Result result = myFunction();
       std::cout << "Value 1: " << result.value1 << std::endl;
       std::cout << "Value 2: " << result.value2 << std::endl;
       return 0;
   }
   ```

2. **使用 `std::tuple`**：C++标准库提供了 `std::tuple`，它是一个通用的类模板，用于将多个值组合在一起。你可以将多个返回值打包到 `std::tuple` 中，然后返回它。

   ```cpp
   #include <tuple>

   std::tuple<int, double> myFunction() {
       return std::make_tuple(42, 3.14);
   }

   int main() {
       std::tuple<int, double> result = myFunction();
       int value1;
       double value2;
       std::tie(value1, value2) = result;
       std::cout << "Value 1: " << value1 << std::endl;
       std::cout << "Value 2: " << value2 << std::endl;
       return 0;
   }
   ```

3. **使用引用参数**：你可以通过函数的参数传递一个或多个引用参数，以在函数内部修改多个值，然后返回单个值，表示操作的成功或失败。这种方式通常用于需要返回状态信息的情况。

   ```cpp
   bool myFunction(int& value1, double& value2) {
       value1 = 42;
       value2 = 3.14;
       return true; // 表示操作成功
   }
   
   int main() {
       int v1;
       double v2;
       if (myFunction(v1, v2)) {
           std::cout << "Value 1: " << v1 << std::endl;
           std::cout << "Value 2: " << v2 << std::endl;
       }
       return 0;
   }
   ```

每种方法都有其用途，你可以根据具体的需求和编程风格选择最适合的方法来处理多个返回值。结构体/类和 `std::tuple` 通常用于返回多个相关的值，而引用参数通常用于返回状态信息和多个值的组合。

## 模板

```c++
template<typename T>
void Print(T value)
{
    std::cout << value << std::endl;
}

int main()
{
    Print<int>(5);
    Print("hello");
    Print(5.5f);
}//模板用来指定类型
```

```c++
template<int size>
class Array
{
private:
    int m_Array[size];
public:
    int GetSize() const { return size; }
};

int main()
{
    Array<5> a{};
    std::cout << a.GetSize() << std::endl;
}//用模板来指定大小
```

C++ 模板是一种强大的编程工具，允许你编写通用代码，以处理不同数据类型的数据，同时保持代码的可重用性。模板可以应用于函数、类、和类的成员函数，允许你在编译时生成不同的特化版本，以满足特定数据类型的需求。下面是有关 C++ 模板的一些重要概念：

1. **函数模板**：函数模板允许你编写一个通用的函数，可以操作不同数据类型的参数。你可以在函数中使用模板参数来表示数据类型，并使用这些参数来执行相同的操作。函数模板使用 `template` 关键字定义，通常在函数声明前声明模板参数。

   ```cpp
   template <typename T>
   T add(T a, T b) {
       return a + b;
   }
   ```

2. **类模板**：类模板允许你创建一个通用的类，可以包含不同数据类型的成员变量和成员函数。类模板使用 `template` 关键字定义，通常在类声明前声明模板参数。

   ```cpp
   template <typename T>
   class MyContainer {
   public:
       MyContainer(T value) : data(value) {}
       T getData() { return data; }
   private:
       T data;
   };
   ```

3. **模板参数**：模板参数是你在模板中定义的占位符，表示特定数据类型。可以使用 `typename` 或 `class` 关键字定义模板参数。

4. **模板特化**：你可以创建模板的特化版本，以处理特定数据类型的需求。模板特化允许你为某些数据类型提供自定义实现。

   ```cpp
   template <>
   double add(double a, double b) {
       return a + b + 1.0;
   }
   ```

5. **模板函数的实例化**：模板函数在编译时根据具体的数据类型实例化。当你调用模板函数时，编译器会根据参数的数据类型生成适当的函数。

   ```cpp
   int result = add(5, 3); // 编译器生成 add<int>(5, 3)
   ```

6. **模板类的实例化**：模板类也在编译时根据具体的数据类型实例化。你可以声明一个特定数据类型的对象来实例化模板类。

   ```cpp
   MyContainer<int> container(42);
   ```

7. **非类型模板参数**：除了表示数据类型的模板参数，C++ 还支持非类型模板参数，它们可以表示常量值、枚举、整数等。

   ```cpp
   template <int N>
   struct Array {
       int data[N];
   };
   ```

C++ 模板提供了强大的泛型编程能力，使你能够编写通用的代码，从而提高代码的重用性和可维护性。模板在标准库中广泛使用，例如 STL（标准模板库）中的容器、算法等都是基于模板的。

## 堆和栈的区别

在计算机科学和编程中，"堆"（Heap）和"栈"（Stack）是两个不同的内存区域，用于存储数据和管理程序的执行。

以下是堆和栈的主要区别：

1. **内存分配方式**：
   - **栈**：栈是一种自动内存分配的数据结构，其管理方式是“先进后出”（Last In, First Out，LIFO）。栈内存由编译器自动分配和释放，用于存储局部变量、函数调用和返回地址等。栈的分配和释放速度非常快，但大小通常有限。
   - **堆**：堆是一种手动内存分配的数据结构，程序员需要显式地分配和释放堆内存。堆内存通常用于存储动态分配的数据，例如通过 `new`（C++）或 `malloc`（C）分配的内存。堆内存的大小通常比栈大，但它需要手动管理，否则可能导致内存泄漏或内存溢出。

2. **生存期**：
   - **栈**：栈中的变量的生存期与其包含它的函数的调用周期相同。当函数退出时，栈上的局部变量将自动销毁。
   - **堆**：堆上的数据的生存期由程序员控制，它们可以在多个函数之间传递，直到被显式释放。如果不释放堆内存，可能会导致内存泄漏。

3. **分配速度**：
   - **栈**：栈的分配和释放速度通常非常快，因为只需要移动栈指针。
   - **堆**：堆的分配和释放速度较慢，因为需要在堆内存中查找可用的空间，并在分配时进行内部管理。

4. **使用情况**：
   - **栈**：栈用于存储局部变量、函数调用和递归。
   - **堆**：堆用于存储动态分配的数据，例如动态数组、对象等，以及需要在函数调用之间共享的数据。

总之，堆和栈都有各自的用途和特性。栈用于快速的自动分配和释放内存，适用于局部变量和函数调用。堆用于动态分配和控制内存的情况，适用于需要长时间存活或在多个函数之间传递的数据。正确理解和使用堆和栈内存是编写高效和健壮的程序的重要部分。

## 宏

在C++和许多其他编程语言中，宏（Macro）通常指的是预处理宏，它是一种文本替换机制，允许程序员定义一些代码片段，并在编译时进行文本替换。C++中的宏是使用预处理器指令 `#define` 来定义的。宏可以用于定义常量、简化代码、进行代码生成等各种用途。

以下是一些关于宏的常见用法和特点：

1. **定义常量**：你可以使用宏来定义常量值，以便在代码中引用。这些宏在编译时会被替换为相应的值。

   ```cpp
   #define PI 3.14159265359
   ```

   在代码中可以使用 `PI` 来代替 `3.14159265359`。

2. **宏函数**：宏可以像函数一样接受参数，并在替换时将参数值嵌入到宏的定义中。这可以用于创建类似函数的代码。

   ```cpp
   #define SQUARE(x) ((x) * (x))
   ```

   在代码中可以使用 `SQUARE(5)` 来计算 5 的平方。

3. **条件编译**：宏经常用于条件编译，通过 `#ifdef` 和 `#ifndef` 等指令控制编译时包含或排除特定的代码块。

   ```c++
   #define DEBUG

   #ifdef DEBUG
   // 调试代码
   #endif
   ```

4. **代码生成**：宏可以用于生成重复的代码片段，减少重复劳动。

   ```c++
   #define CREATE_VARIABLES(name, type) \
       type name ## 1; \
       type name ## 2; \
       type name ## 3;

   CREATE_VARIABLES(myVar, int)
   ```

   这将生成变量 `myVar1`, `myVar2`, 和 `myVar3`。

5. **注意事项**：
   - 使用宏时要小心，因为它们是简单的文本替换，可能会导致不符合预期的结果。
   - 宏名称通常使用全大写字母，以区分宏和变量。
   - 尽量避免复杂的宏定义，以提高代码的可读性和维护性。
   - 在C++中，通常更推荐使用 `const` 或 `constexpr` 来定义常量，而不是宏，因为这些方式更类型安全和易于调试。

总之，宏是一种强大的文本替换工具，用于在编译时修改源代码。但要谨慎使用宏，以确保代码的可读性和可维护性。在C++中，通常可以使用更安全和类型友好的替代方案。

## auto

`auto` 是 C++11 标准引入的关键字，用于自动推断变量的类型。它允许编译器根据初始化表达式的类型来确定变量的数据类型，从而简化了类型声明和提高了代码的可读性。`auto` 变量通常用于提高代码的可维护性和减少冗余的类型信息。

以下是一些关于 `auto` 的使用示例和注意事项：

1. **基本用法**：使用 `auto` 声明变量时，编译器会根据初始化表达式的类型确定变量的类型。

   ```cpp
   auto x = 42; // x 被推断为 int
   auto y = 3.14; // y 被推断为 double
   auto name = "John"; // name 被推断为 const char*
   ```

2. **迭代器和范围循环**：`auto` 在遍历容器和使用范围循环时特别有用，因为它可以让你不必显式指定容器的数据类型。

   ```cpp
   std::vector<int> numbers = {1, 2, 3, 4, 5};
   for (auto it = numbers.begin(); it != numbers.end(); ++it) {
       // 使用 auto 推断的迭代器类型
       int value = *it;
   }

   for (const auto& num : numbers) {
       // 使用 auto 推断的元素类型
       std::cout << num << " ";
   }
   ```

3. **函数返回值类型**：你可以使用 `auto` 来推断函数的返回值类型。这在返回值类型较复杂或依赖于参数的情况下特别有用。

   ```cpp
   auto add(int a, int b) {
       return a + b; // 返回类型将被推断为 int
   }
   ```

4. **lambda 表达式**：lambda 表达式经常使用 `auto` 来简化函数对象的类型声明。

   ```cpp
   auto add = [](int a, int b) { return a + b; }; // 使用 auto 推断类型
   ```

5. **注意事项**：
   - 使用 `auto` 时要确保初始化表达式提供了足够的信息以进行正确的类型推断。
   - `auto` 变量的类型在编译时确定，因此不会带来运行时性能开销。
   - 虽然 `auto` 简化了代码，但过度使用它可能会降低代码的可读性，因此要谨慎使用。

总之，`auto` 是一种方便的工具，可以在不失去类型安全的前提下简化代码。它可以减少冗余的类型信息，提高代码的可读性，尤其在遍历容器、使用lambda表达式和处理复杂函数返回值类型时非常有用。

## 静态数组

在 C++ 中，静态数组是一种包含固定数量元素的数组，这些元素在编译时分配并在程序执行期间保持不变。静态数组的大小通常在定义时确定，不能在运行时更改。

以下是一些关于静态数组的基本知识和示例：

1. **声明和初始化静态数组**：

   静态数组的声明和初始化可以通过以下方式完成：

   ```cpp
   // 声明一个整数数组，大小为5
   int myArray[5];

   // 初始化数组的同时赋值
   int numbers[] = {1, 2, 3, 4, 5};

   // 如果不显式提供大小，编译器将根据初始化的元素数目确定数组大小
   char vowels[] = {'a', 'e', 'i', 'o', 'u'};
   ```

2. **访问数组元素**：

   数组元素可以通过下标（从0开始）来访问：

   ```cpp
   int x = myArray[2]; // 获取第3个元素的值
   ```

3. **数组大小**：

   静态数组的大小在声明时确定，你可以使用 `sizeof` 运算符来获取数组的大小（以字节为单位）：

   ```cpp
   int size = sizeof(numbers) / sizeof(numbers[0]); // 获取数组的元素数量
   ```

4. **数组边界检查**：

   注意，在C++中，静态数组没有内置的边界检查。访问超出数组界限的元素会导致未定义的行为，因此要小心避免越界访问。

   ```cpp
   int value = myArray[10]; // 可能导致未定义行为
   ```

5. **多维数组**：

   C++支持多维静态数组，例如二维数组。声明和访问多维数组的方式如下：

   ```cpp
   int matrix[3][3] = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
   int element = matrix[1][2]; // 获取第2行第3列的元素
   ```

6. **静态数组的限制**：

   静态数组的大小必须在编译时确定，因此无法用于需要动态大小的数据结构。如果需要动态大小的数组，可以考虑使用动态分配的内存，如 `new` 和 `delete`（C++）或 `malloc` 和 `free`（C）来创建和释放内存。

总之，静态数组是一种在编译时确定大小的数组，适用于包含已知数量元素的情况。要小心避免越界访问，并考虑在需要动态大小的情况下使用动态内存分配。

`std::array` 是 C++ 标准库中提供的一种容器，用于表示固定大小的数组。它是一个更安全、更类型安全的替代品，相比于原始的静态数组，提供了一些额外的功能和好处。`std::array` 提供了以下特点和用法：

1. **固定大小**：`std::array` 具有固定的大小，一旦分配了大小，就不能更改。

2. **类型安全**：与原始数组不同，`std::array` 提供了类型安全，不允许越界访问。

3. **容器功能**：`std::array` 拥有许多与标准库容器（如 `std::vector`）相似的功能，包括大小查询、元素访问、范围循环等。

4. **遍历**：可以使用范围循环或迭代器来遍历 `std::array` 中的元素。

5. **快速访问**：通过下标访问元素与原始数组相似，但更安全。

以下是一个简单的示例，演示了如何使用 `std::array`：

```cpp
#include <iostream>
#include <array>

int main() {
    // 创建一个包含5个整数的 std::array
    std::array<int, 5> myArray = {1, 2, 3, 4, 5};

    // 访问元素
    int value = myArray[2]; // 获取第3个元素

    // 使用范围循环遍历元素
    for (const int& element : myArray) {
        std::cout << element << " ";
    }

    // 查询数组大小
    std::cout << "Size: " << myArray.size() << std::endl;

    return 0;
}
```

请注意，`std::array` 是固定大小的数组，因此如果你需要动态大小的数组，应该使用 `std::vector`。`std::array` 提供了静态数组的安全性和方便性，但大小在编译时确定。

## 函数指针

函数指针是指针，它指向函数而不是普通的数据。在 C++ 和 C 语言中，函数指针允许你存储和传递函数的地址，从而实现更灵活的函数调用和回调机制。函数指针通常用于以下情况：

1. **回调函数**：函数指针可以作为参数传递给其他函数，允许你在调用时指定要执行的具体函数。

2. **动态函数调用**：根据条件或用户输入，可以选择调用不同的函数，而不需要使用条件语句。

3. **函数指针数组**：你可以创建一个包含函数指针的数组，以便在数组中存储和调用不同的函数。

以下是关于函数指针的一些基本用法：

**声明函数指针**

要声明函数指针，需要指定函数的返回类型和参数类型。例如，假设有一个函数 `int add(int a, int b)`，可以声明一个函数指针来指向它：

```cpp
int (*functionPointer)(int, int);
```

这个声明告诉编译器，`functionPointer` 是一个指向接受两个整数参数并返回整数的函数的指针。

**赋值和使用函数指针**

要将函数指针指向特定的函数，可以将函数的名称分配给函数指针，或者使用地址运算符 `&`：

```cpp
functionPointer = add; // 或者 functionPointer = &add;
```

现在，`functionPointer` 指向 `add` 函数。你可以使用函数指针来调用函数：

```cpp
int result = functionPointer(3, 4); // 调用 add(3, 4) 并将结果存储在 result 中
```

**函数指针作为参数**

函数指针可以作为函数的参数，这通常用于实现回调函数的机制。例如，以下是一个接受函数指针作为参数的函数，它可以执行回调函数：

```cpp
void performOperation(int a, int b, int (*operation)(int, int)) {
    int result = operation(a, b);
    std::cout << "Result: " << result << std::endl;
}
```

你可以将 `add` 函数的指针传递给 `performOperation`：

```cpp
int add(int a, int b) {
    return a + b;
}

int main() {
    performOperation(3, 4, add); // 执行 add 函数并输出结果
    return 0;
}
```

这个示例中，`performOperation` 接受一个函数指针作为参数，并使用该函数指针来执行回调函数。

函数指针是一种强大的工具，它可以实现许多高级的编程技术，如回调函数、动态函数调用和插件系统。然而，要小心使用函数指针，确保类型匹配和安全性，以避免潜在的运行时错误。在现代 C++ 中，函数指针的使用也可以与 C++11 引入的 `std::function` 和 `std::bind` 等功能一起考虑，以实现更灵活和类型安全的回调机制。

## 优先队列

```cpp
bool operator()(const Node &a, const Node &b) const {
    return a.f > b.f;  // 如果 a.f > b.f，则 a 的优先级比 b 低
}

std::priority_queue<Node, std::vector<Node>, NodeCompare> pq;//小值优先

    pq.push({3});
    pq.push({1});
    pq.push({4});

    while (!pq.empty())
    {
        std::cout << pq.top().f << " "; // 输出：1 3 4
        pq.pop();
    }
```

