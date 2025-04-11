#pragma once

#ifdef _MSC_VER
#define REFLECT_CXX_VERSION _MSVC_LANG
#else
#define REFLECT_CXX_VERSION __cplusplus
#endif

#if REFLECT_CXX_VERSION < 202000L
#error "Need a c++20 compiler"
#endif

#include <json.hpp>
#include <set>
#include <tuple>
#include <typeindex>
#include <typeinfo>
#include <utility>

namespace toolkit {

namespace _Reflect_Imp {

template <typename T> struct field_info {
  const char *name; // name for the field
  T ptr;            // pointer to the field
};

// traverse the tuple
template <typename Tuple, typename Func, std::size_t... I>
void tuple_for_each_impl(Tuple &&t, Func &&f, std::index_sequence<I...>) {
  (f(std::get<I>(t)), ...);
}

class class_registry {
private:
  std::set<std::size_t> registered_classes;
  class_registry() = default;

public:
  static inline class_registry &instance() {
    static class_registry registry;
    return registry;
  }

  template <typename T> void register_class() {
    registered_classes.insert(typeid(T).hash_code());
  }

  template <typename T> bool contains() const {
    return (registered_classes.count(typeid(T).hash_code()) != 0);
  }

  bool contains(std::size_t index) const {
    return (registered_classes.count(index) != 0);
  }
};

}; // namespace _Reflect_Imp

#define CONCAT_IMPL(x, y) x##y
#define CONCAT(x, y) CONCAT_IMPL(x, y)

#define REFLECT_FOR_EACH_0(class_name, ACTION)
#define REFLECT_FOR_EACH_1(class_name, ACTION, X, ...) ACTION(class_name, X)
#define REFLECT_FOR_EACH_2(class_name, ACTION, X, ...)                         \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 1)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_3(class_name, ACTION, X, ...)                         \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 2)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_4(class_name, ACTION, X, ...)                         \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 3)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_5(class_name, ACTION, X, ...)                         \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 4)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_6(class_name, ACTION, X, ...)                         \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 5)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_7(class_name, ACTION, X, ...)                         \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 6)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_8(class_name, ACTION, X, ...)                         \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 7)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_9(class_name, ACTION, X, ...)                         \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 8)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_10(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 9)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_11(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 10)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_12(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 11)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_13(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 12)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_14(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 13)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_15(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 14)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_16(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 15)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_17(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 16)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_18(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 17)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_19(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 18)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_20(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 19)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_21(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 20)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_22(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 21)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_23(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 22)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_24(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 23)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_25(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 24)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_26(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 25)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_27(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 26)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_28(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 27)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_29(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 28)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_30(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 29)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_31(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 30)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_32(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 31)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_33(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 32)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_34(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 33)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_35(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 34)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_36(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 35)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_37(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 36)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_38(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 37)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_39(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 38)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_40(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 39)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_41(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 40)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_42(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 41)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_43(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 42)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_44(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 43)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_45(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 44)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_46(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 45)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_47(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 46)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_48(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 47)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_49(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 48)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_50(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 49)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_51(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 50)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_52(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 51)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_53(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 52)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_54(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 53)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_55(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 54)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_56(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 55)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_57(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 56)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_58(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 57)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_59(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 58)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_60(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 59)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_61(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 60)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_62(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 61)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_63(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 62)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_64(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 63)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_65(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 64)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_66(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 65)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_67(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 66)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_68(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 67)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_69(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 68)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_70(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 69)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_71(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 70)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_72(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 71)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_73(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 72)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_74(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 73)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_75(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 74)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_76(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 75)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_77(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 76)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_78(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 77)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_79(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 78)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_80(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 79)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_81(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 80)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_82(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 81)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_83(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 82)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_84(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 83)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_85(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 84)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_86(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 85)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_87(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 86)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_88(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 87)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_89(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 88)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_90(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 89)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_91(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 90)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_92(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 91)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_93(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 92)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_94(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 93)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_95(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 94)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_96(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 95)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_97(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 96)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_98(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 97)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_99(class_name, ACTION, X, ...)                        \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 98)(class_name, ACTION, __VA_ARGS__)
#define REFLECT_FOR_EACH_100(class_name, ACTION, X, ...)                       \
  ACTION(class_name, X),                                                       \
      CONCAT(REFLECT_FOR_EACH_, 99)(class_name, ACTION, __VA_ARGS__)

#define FOR_EACH_NARG(...)                                                     \
  FOR_EACH_NARG_IMPL(                                                          \
      __VA_ARGS__, 100, 99, 98, 97, 96, 95, 94, 93, 92, 91, 90, 89, 88, 87,    \
      86, 85, 84, 83, 82, 81, 80, 79, 78, 77, 76, 75, 74, 73, 72, 71, 70, 69,  \
      68, 67, 66, 65, 64, 63, 62, 61, 60, 59, 58, 57, 56, 55, 54, 53, 52, 51,  \
      50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33,  \
      32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15,  \
      14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0)
#define FOR_EACH_NARG_IMPL(                                                    \
    _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16,     \
    _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, \
    _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, _45, _46, \
    _47, _48, _49, _50, _51, _52, _53, _54, _55, _56, _57, _58, _59, _60, _61, \
    _62, _63, _64, _65, _66, _67, _68, _69, _70, _71, _72, _73, _74, _75, _76, \
    _77, _78, _79, _80, _81, _82, _83, _84, _85, _86, _87, _88, _89, _90, _91, \
    _92, _93, _94, _95, _96, _97, _98, _99, _100, N, ...)                      \
  N

#define REFLECT_FOR_EACH(class_name, ACTION, ...)                              \
  CONCAT(REFLECT_FOR_EACH_, FOR_EACH_NARG(__VA_ARGS__))                        \
  (class_name, ACTION, __VA_ARGS__)

#define MAKE_REFLECT_FIELD(class_name, field)                                  \
  toolkit::_Reflect_Imp::field_info<decltype(&class_name::field)> {            \
    #field, &class_name::field                                                 \
  }

/**
 * Traverse one tuple.
 *
 * for_each(t, [&](auto e) {...});
 */
template <typename Tuple, typename Func> void for_each(Tuple &&t, Func &&f) {
  toolkit::_Reflect_Imp::tuple_for_each_impl(
      std::forward<Tuple>(t), std::forward<Func>(f),
      std::make_index_sequence<std::tuple_size_v<std::decay_t<Tuple>>>{});
}

template <typename T, typename = void> struct reflected : std::false_type {};
template <typename T>
struct reflected<T, std::void_t<typename T::_Reflectable_Flag_>>
    : std::true_type {};

/**
 * Decide whether a class is reflectable or not.
 */
template <typename T> bool is_reflectable() {
  return toolkit::_Reflect_Imp::class_registry::instance().contains<T>();
}
#define IS_REFLECTABLE_TYPE(var)                                               \
  (toolkit::_Reflect_Imp::class_registry::instance().contains(                 \
      typeid(decltype(a)).hash_code()))

/**
 * Reflect members of a class, this macro should be placed in the body of a
 * class.
 *
 * The reflected class must be trivially constructible, if there's a custom
 * constructor, all parameters for this constructor should have a default value.
 *
 * The parameter `class_name` should come with it's namespace if exists, for
 * example `toolkit::SomeClass`.
 *
 * To successfully serialize and deserialize the class, all the member variables
 * within the macro should have complete serilization definition for
 * `nlohmann::json`.
 */
#define REFLECT(class_name, ...)                                               \
private:                                                                       \
  static inline bool _register_reflect_##class_name = []() {                   \
    toolkit::_Reflect_Imp::class_registry::instance()                          \
        .register_class<class_name>();                                         \
    return true;                                                               \
  }();                                                                         \
                                                                               \
public:                                                                        \
  using _Reflectable_Flag_ = std::true_type;                                   \
  static auto get_reflect_fields() {                                           \
    return std::make_tuple(__VA_OPT__(                                         \
        REFLECT_FOR_EACH(class_name, MAKE_REFLECT_FIELD, __VA_ARGS__)));       \
  }                                                                            \
  friend void to_json(nlohmann::json &j, const class_name &obj) {              \
    j = obj.serialize();                                                       \
  }                                                                            \
  friend void from_json(const nlohmann::json &j, class_name &obj) {            \
    obj.deserialize(j);                                                        \
  }                                                                            \
  nlohmann::json serialize() const {                                           \
    nlohmann::json j;                                                          \
    toolkit::for_each(get_reflect_fields(), [&](auto field) {                  \
      using FieldType = std::decay_t<decltype(this->*(field.ptr))>;            \
      j[field.name] = this->*(field.ptr);                                      \
    });                                                                        \
    return j;                                                                  \
  }                                                                            \
  void deserialize(const nlohmann::json &j) {                                  \
    toolkit::for_each(get_reflect_fields(), [&](auto field) {                  \
      using FieldType = std::decay_t<decltype(this->*(field.ptr))>;            \
      if (j.contains(field.name))                                              \
        this->*(field.ptr) = j[field.name].get<FieldType>();                   \
    });                                                                        \
  }                                                                            \
  template <typename Func> void for_each_reflect_field(Func &&f) {             \
    toolkit::for_each(get_reflect_fields(), f);                                \
  }                                                                            \
  static std::string get_reflect_name() { return #class_name; }

}; // namespace toolkit