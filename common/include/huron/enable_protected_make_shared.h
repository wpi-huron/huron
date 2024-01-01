#pragma once

#include <memory>

namespace huron {

/**
 * @brief This class provides a static method to create a shared_ptr to a class
 * with a protected constructor.
 *
 * This is useful for classes that should not be instantiated directly, but
 * should instead be created by a factory method.
 *
 * Example:
 * class Foo : public enable_protected_make_shared<Foo> {
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
 *   std::shared_ptr<Foo> CreateFoo(int a, double b) {
 *     return Foo::make_shared(a, b);
 *   }
 * }
 *
 * @ref https://stackoverflow.com/a/73236821
 */
template <typename ClassWithProtectedCtor>
class enable_protected_make_shared {
 protected:
  template <typename... Args>
  static std::shared_ptr<ClassWithProtectedCtor> make_shared(Args &&... args) {
    class make_shared_enabler : public ClassWithProtectedCtor {
     public:
      // Ensures that the constructor is not public.
      static_assert(!std::is_constructible_v<ClassWithProtectedCtor, Args...>);
      make_shared_enabler(Args &&... args)
        : ClassWithProtectedCtor(std::forward<Args>(args)...) {}
    };
    return std::make_shared<make_shared_enabler>(std::forward<Args>(args)...);
  }
};

}  // namespace huron
