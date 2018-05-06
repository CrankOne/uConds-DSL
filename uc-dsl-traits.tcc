# ifndef _H_UCONDSL_TRAITS_H_
# define _H_UCONDSL_TRAITS_H_

# include "uc-dsl-common.hpp"

namespace ucdslang {

namespace _impl {
template<typename AbstractFilterT> class VCtrRegistry;  // fwd
}  // namespace _impl

/// Traits defined for some particular subject. May be specialized.
template<typename SubjectT>
struct Traits {
    /// Type of subject to be filtered.
    typedef SubjectT Subject;
    /// Result type returned by applying filtering expression to subject.
    typedef bool Result;
    /// Parameterized filter template class used to build logical expressions.
    template<typename T> using Parameterized = _impl::ParameterizedFilter<T, Subject, Result>;
    /// A polymorphic base class for filters. Must be base of the Parameterized.
    typedef _impl::AbstractFilter<Subject, Result> AbstractFilter;
    /// Handle referencing particular filter within expression tree.
    typedef std::shared_ptr<AbstractFilter> NodeRef;
    /// Expression tree.
    typedef ExpressionTree<NodeRef> Tree;

    /// Factory method for ctring the filters.
    static AbstractFilter * produce( const std::string
                         , const AST &
                         , std::ostream * logStream=nullptr );
};  // struct Traits

template<typename T>
typename Traits<T>::AbstractFilter *
Traits<T>::produce( const std::string name
                  , const AST & ast
                  , std::ostream * ssPtr ) {
    auto ctr = _impl::VCtrRegistry<AbstractFilter>::constructor(name);
    if( nullptr == ctr ) {
        throw std::runtime_error( "No filter named \""
                + name
                + "\" is registered." );
    }
    if( ssPtr ) {
        *ssPtr << "The " << (void *) ctr << " will construct \""
               << name << "\" instance w/ parameters: "
               << ast << std::endl;
    }
    auto r = _impl::VCtrRegistry<AbstractFilter>::construct(ctr, ast);
    assert(r);  // constructor returned NULL ptr
    return r;
}

}  // namespace ucdslang

# endif  // _H_UCONDSL_TRAITS_H_

