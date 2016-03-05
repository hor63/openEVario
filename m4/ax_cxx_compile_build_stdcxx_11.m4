# ============================================================================
#  Local
# ============================================================================
#
# SYNOPSIS
#
#   AX_CXX_COMPILE_BUILD_STDCXX_11([ext|noext],[mandatory|optional])
#
# DESCRIPTION
#
#   This file is 95% copied from ax_cxx_compile_stdcxx_11.m4 from autoconf_archive.
#   Changes are mostly renaming variables and defines, and pushing and popping define names
#   to the _FOR_BUILD equivalents
#
#   Check for baseline language coverage in the build compiler for the C++11
#   standard; if necessary, add switches to CXXFLAGS_FOR_BUILD to enable
#   support.
#
#   The first argument, if specified, indicates whether you insist on an
#   extended mode (e.g. -std=gnu++11) or a strict conformance mode (e.g.
#   -std=c++11).  If neither is specified, you get whatever works, with
#   preference for an extended mode.
#
#   The second argument, if specified 'mandatory' or if left unspecified,
#   indicates that baseline C++11 support is required and that the macro
#   should error out if no mode with that support is found.  If specified
#   'optional', then configuration proceeds regardless, after defining
#   HAVE_CXX11 if and only if a supporting mode is found.
#
# LICENSE
#
#   Copyright (c) 2016 Kai Horstmann <horstmannkai@hotmail.com>
#
#   Copying and distribution of this file, with or without modification, are
#   permitted in any medium without royalty provided the copyright notice
#   and this notice are preserved. This file is offered as-is, without any
#   warranty.

#serial 1

m4_define([_AX_CXX_COMPILE_BUILD_STDCXX_11_testbody], [[
  template <typename T>
    struct check
    {
      static_assert(sizeof(int) <= sizeof(T), "not big enough");
    };

    struct Base {
    virtual void f() {}
    };
    struct Child : public Base {
    virtual void f() override {}
    };

    typedef check<check<bool>> right_angle_brackets;

    int a;
    decltype(a) b;

    typedef check<int> check_type;
    check_type c;
    check_type&& cr = static_cast<check_type&&>(c);

    auto d = a;
    auto l = [](){};

    // http://stackoverflow.com/questions/13728184/template-aliases-and-sfinae
    // Clang 3.1 fails with headers of libstd++ 4.8.3 when using std::function because of this
    namespace test_template_alias_sfinae {
        struct foo {};

        template<typename T>
        using member = typename T::member_type;

        template<typename T>
        void func(...) {}

        template<typename T>
        void func(member<T>*) {}

        void test();

        void test() {
            func<foo>(0);
        }
    }
]])

AC_DEFUN([AX_CXX_COMPILE_BUILD_STDCXX_11], [dnl
  m4_if([$1], [], [],
        [$1], [ext], [],
        [$1], [noext], [],
        [m4_fatal([invalid argument `$1' to AX_CXX_COMPILE_BUILD_STDCXX_11])])dnl
  m4_if([$2], [], [ax_cxx_compile_build_cxx11_required=true],
        [$2], [mandatory], [ax_cxx_compile_build_cxx11_required=true],
        [$2], [optional], [ax_cxx_compile_build_cxx11_required=false],
        [m4_fatal([invalid second argument `$2' to AX_CXX_COMPILE_BUILD_STDCXX_11])])


dnl Use the standard macros, but make them use other variable names
dnl
pushdef([CXX], CXX_FOR_BUILD)dnl
pushdef([CXXCPP], CXXCPP_FOR_BUILD)dnl
pushdef([CXXFLAGS], CXXFLAGS_FOR_BUILD)dnl
pushdef([CPPFLAGS], CPPFLAGS_FOR_BUILD)dnl
pushdef([CXXCPPFLAGS], CXXCPPFLAGS_FOR_BUILD)dnl

  AC_LANG_PUSH([C++])dnl
  ac_success=no
  AC_CACHE_CHECK(whether $CXX supports C++11 features by default,
  ax_cv_cxx_compile_build_cxx11,
  [AC_COMPILE_IFELSE([AC_LANG_SOURCE([_AX_CXX_COMPILE_BUILD_STDCXX_11_testbody])],
    [ax_cv_cxx_compile_build_cxx11=yes],
    [ax_cv_cxx_compile_build_cxx11=no])])
  if test x$ax_cv_cxx_compile_build_cxx11 = xyes; then
    ac_success=yes
  fi

  m4_if([$1], [noext], [], [dnl
  if test x$ac_success = xno; then
    for switch in -std=gnu++11 -std=gnu++0x; do
      cachevar=AS_TR_SH([ax_cv_cxx_compile_build_cxx11_$switch])
      AC_CACHE_CHECK(whether $CXX supports C++11 features with $switch,
                     $cachevar,
        [ac_save_CXXFLAGS="$CXXFLAGS"
         CXXFLAGS="$CXXFLAGS $switch"
         AC_COMPILE_IFELSE([AC_LANG_SOURCE([_AX_CXX_COMPILE_BUILD_STDCXX_11_testbody])],
          [eval $cachevar=yes],
          [eval $cachevar=no])
         CXXFLAGS="$ac_save_CXXFLAGS"])
      if eval test x\$$cachevar = xyes; then
        CXXFLAGS="$CXXFLAGS $switch"
        ac_success=yes
        break
      fi
    done
  fi])

  m4_if([$1], [ext], [], [dnl
  if test x$ac_success = xno; then
    for switch in -std=c++11 -std=c++0x; do
      cachevar=AS_TR_SH([ax_cv_cxx_compile_build_cxx11_$switch])
      AC_CACHE_CHECK(whether $CXX supports C++11 features with $switch,
                     $cachevar,
        [ac_save_CXXFLAGS="$CXXFLAGS"
         CXXFLAGS="$CXXFLAGS $switch"
         AC_COMPILE_IFELSE([AC_LANG_SOURCE([_AX_CXX_COMPILE_BUILD_STDCXX_11_testbody])],
          [eval $cachevar=yes],
          [eval $cachevar=no])
         CXXFLAGS="$ac_save_CXXFLAGS"])
      if eval test x\$$cachevar = xyes; then
        CXXFLAGS="$CXXFLAGS $switch"
        ac_success=yes
        break
      fi
    done
  fi])
  AC_LANG_POP([C++])
  if test x$ax_cxx_compile_build_cxx11_required = xtrue; then
    if test x$ac_success = xno; then
      AC_MSG_ERROR([*** A compiler with support for C++11 language features is required.])
    fi
  else
    if test x$ac_success = xno; then
      HAVE_BUILD_CXX11=0
      AC_MSG_NOTICE([No compiler with C++11 support was found])
    else
      HAVE_CXX11=1
      AC_DEFINE(HAVE_BUILD_CXX11,1,
                [define if the compiler supports basic C++11 syntax])
    fi

    AC_SUBST(HAVE_BUILD_CXX11)
  fi

popdef([CXXCPPFLAGS])dnl
popdef([CPPFLAGS])dnl
popdef([CXXFLAGS])dnl
popdef([CXXCPP])dnl
popdef([CXX])dnl


])
