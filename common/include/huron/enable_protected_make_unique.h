#pragma once

#include <memory>

namespace huron {

/**
 * @brief This class provides a static method to create a unique_ptr to a class
 * with a protected constructor.
 *
 * This is useful for classes that should not be instantiated directly, but
 * should instead be created by a factory method.
 *
 * Example:
 * class Foo : public enable_protected_make_unique<Foo> {
 *  public:
 *    friend class Bar;
 *
 *  protected:
 *   Foo(int a, int b) : a_(a), b_(b) {}
 *   int a_;
 *   int b_;
 * };
 *
 * class Bar {
 *   std::unique_ptr<Foo> CreateFoo(int a, double b) {
 *     return Foo::make_unique(a, b);
 *   }
 * }
 *
 * @ref https://stackoverflow.com/a/73236821
 */
template <typename ClassWithProtectedCtor>
class enable_protected_make_unique {
 protected:
  template <typename... Args>
  static std::unique_ptr<ClassWithProtectedCtor> make_unique(Args &&... args) {
    class make_unique_enabler : public ClassWithProtectedCtor {
     public:
      // Ensures that the constructor is not public.
      static_assert(!std::is_constructible_v<ClassWithProtectedCtor, Args...>);
      make_unique_enabler(Args &&... args)
        : ClassWithProtectedCtor(std::forward<Args>(args)...) {}
    };
    return std::make_unique<make_unique_enabler>(std::forward<Args>(args)...);
  }
};

}  // namespace huron
