# ifndef _H_UCONDSL_CONSTRUCTING_H_
# define _H_UCONDSL_CONSTRUCTING_H_

# include "uc-dsl-tree.tcc"
# include "uc-dsl-traits.tcc"

# include <boost/variant/static_visitor.hpp>

# include <unordered_map>

namespace ucdslang {
namespace helpers {

/// Use this visitor to construct expressions within user-side constructors.
template< typename T
        , typename ValueT
        , typename ValueParserT>
class ConstructingVisitor /*: public boost::static_visitor<typename Traits<T>::AbstractFilter *>*/ {
public:
    typedef Traits<T> FT;
    typedef typename FT::template Parameterized<ValueT>
                       ::DecisionTree
                       ::Branch result_type;
private:
    /// Must support operator() and return obj of type Leaf
    ValueParserT _vParse;
public:
    ConstructingVisitor( ValueParserT vp ) : _vParse(vp) {}

    result_type operator()( const Nil & p ) {
        throw std::runtime_error( "Nil expression in AST tree." );
    }

    result_type operator()( const std::pair<Predicate, std::string> & p ) {
        // NOTE: trimming here may be an overkill (would it be better to tune
        // our lexer instead?)
        //boost::algorithm::trim_copy(p.second);
        return _vParse( p.first, p.second );
    }

    result_type operator()( const AST & ast ) {
        return boost::apply_visitor(*this, ast.root);
    }

    result_type operator()( const typename AST::AND & bo ) {
        typedef typename FT::template Parameterized<ValueT>::DecisionTree::AND AND;
        return AND{ boost::apply_visitor( *this, bo.l.root )
                  , boost::apply_visitor( *this, bo.r.root ) };
    }

    result_type operator()( const typename AST::OR & bo ) {
        typedef typename FT::template Parameterized<ValueT>::DecisionTree::OR OR;
        return OR{ boost::apply_visitor( *this, bo.l.root )
                 , boost::apply_visitor( *this, bo.r.root ) };
    }
};

}  // namespace helpers


namespace _impl {

/// Default constructors registry.
template<typename AbstractFilterT>
class VCtrRegistry {
public:
    /// Filter construction callback.
    typedef AbstractFilterT * (*FilterConstructor)( const AST & );
    /// Returns filter constructor or NULL.
    static FilterConstructor constructor( const std::string );
    /// Helper method performing AST treatment during ctrion of new filter instance.
    static AbstractFilterT * construct( FilterConstructor, const AST & );
    /// Registers new ctr making it available within the parsing routines.
    static void register_constructor( const char *, FilterConstructor );
private:
    /// Known constructors.
    static std::unordered_map<std::string, FilterConstructor> * _constructors;
};

template<typename T>
std::unordered_map<std::string, typename VCtrRegistry<T>::FilterConstructor>
    * VCtrRegistry<T>::_constructors = nullptr;

template<typename T> void
VCtrRegistry<T>::register_constructor( const char * name
                                     , typename VCtrRegistry<T>::FilterConstructor ctr ) {
    if( !_constructors ) {
        _constructors = new std::unordered_map< std::string
                                              , typename VCtrRegistry<T>::FilterConstructor>;
    }
    auto rc = _constructors->emplace( name, ctr );
    if( ! rc.second ) {
        throw std::runtime_error( "Repeatative insertion of filter constructor"
                " \"" + std::string(name) + "\".");
    }
}

template<typename T> typename VCtrRegistry<T>::FilterConstructor
VCtrRegistry<T>::constructor( const std::string name ) {
    assert(_constructors);
    auto it = _constructors->find(name);
    if( _constructors->end() == it ) {
        return nullptr;
    }
    return it->second;
}

template<typename T> T *
VCtrRegistry<T>::construct( FilterConstructor ctr, const AST & ast ) {
    assert( ctr );
    return ctr(ast);
}

}  // namespace _impl
}  // namespace ucdslang

# endif  // _H_UCONDSL_CONSTRUCTING_H_

