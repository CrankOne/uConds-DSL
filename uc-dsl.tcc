# ifndef _H_uCONDDSL_H_
# define _H_uCONDDSL_H_

# include <boost/config/warning_disable.hpp>
# include <boost/spirit/include/qi.hpp>
# include <boost/spirit/include/phoenix.hpp>
# include <boost/spirit/include/phoenix_operator.hpp>
# include <boost/phoenix/bind/bind_function.hpp>
# include <boost/phoenix/object/construct.hpp>
# include <boost/fusion/include/std_pair.hpp>
# include <boost/bind.hpp>
# include <boost/algorithm/string/trim.hpp>
# include <boost/move/unique_ptr.hpp>
# include <boost/lexical_cast.hpp>

# include <unordered_map>

namespace ucdslang {

/**\defgroup Basic data types
 * Few basic enums/data types used in uDSL. */
/**@{*/

enum Predicate { unset = 0x0
    , eq  = 0x1, neq = 0x2
    , gt  = 0x3, gte = 0x4
    , lt  = 0x5, lte = 0x6
};

enum BinOpType { AND_, OR_ };

struct Nil {};

std::ostream & operator<<(std::ostream & ss, const Nil & ast);

/**@}*/

/**\defgroup Tree
 * Versatile expression tree structure template routines. */
/**@{*/

template<typename LeafT>
struct ExpressionTree {
    typedef LeafT Leaf;
    template<BinOpType> struct BinaryOperator;  // fwd
    typedef boost::variant< Nil
        , Leaf  // leaf expr node
        , boost::recursive_wrapper< BinaryOperator<AND_> >
        , boost::recursive_wrapper< BinaryOperator<OR_> >
        , boost::recursive_wrapper< ExpressionTree >
        > Branch;

    ExpressionTree & operator&=( const ExpressionTree & );
    ExpressionTree & operator|=( const ExpressionTree & );

    /// Binary operator node: AND/OR for subseq. nodes.
    template<BinOpType BOT> struct BinaryOperator {
        ExpressionTree l, r;
    };

    Branch root;

    template<typename T> ExpressionTree( const T & root_ ) : root(root_) {}
    ExpressionTree() : root(Nil()) {}

    typedef BinaryOperator<AND_> AND;
    typedef BinaryOperator<OR_> OR;
    typedef Branch type;  // xxx?
};

template<typename LeafT> ExpressionTree<LeafT> &
ExpressionTree<LeafT>::operator&=( const ExpressionTree<LeafT> & r ) {
    root = AND{ *this, r };
    return *this;
}

template<typename LeafT> ExpressionTree<LeafT> &
ExpressionTree<LeafT>::operator|=( const ExpressionTree<LeafT> & r ) {
    root = OR{ *this, r };
    return *this;
}

/// AST for interim representation of single-typed values
/// (expressions in "<label> : <expr>" pairs).
typedef ExpressionTree<std::pair<Predicate, std::string> > AST;

/**@}*/

/**\defgroup Printing
 * Tree printing routines. */
/**@{*/

/// Specialize this template to print leaf of certain type.
template<typename LeafT>
struct LeafPrintingTraits {
    static void print_leaf( const LeafT & l, std::ostream & os) { os << "[" << l << "]"; }
};

/// Tree printing reentrant visitor.
template<typename LeafT>
struct ExprTreePrintingVisitor : public boost::static_visitor<void> {
    typedef ExpressionTree<LeafT> Tree;

    std::ostream & ss;
    ExprTreePrintingVisitor( std::ostream & ss_ ) : ss(ss_) {}

    void operator()( const Nil & p ) {
        ss << "(Nil)";
    }

    void operator()( const LeafT & leaf ) {
        LeafPrintingTraits<LeafT>::print_leaf( leaf, ss );
    }

    void operator()( const Tree & ast ) {
        ss << "{ ";
        boost::apply_visitor( *this, ast.root );
        ss << " }";
    }

    void operator()( const typename Tree::template BinaryOperator<AND_> & bo ) {
        ss << "(";
        boost::apply_visitor( *this, bo.l.root );
        ss << " and ";
        boost::apply_visitor( *this, bo.r.root );
        ss << ")";
    }

    void operator()( const typename Tree::template BinaryOperator<OR_> & bo ) {
        ss << "(";
        boost::apply_visitor( *this, bo.l.root );
        ss << " or ";
        boost::apply_visitor( *this, bo.r.root );
        ss << ")";
    }
};

template<typename LeafT>
std::ostream & operator<<(std::ostream & ss, ExpressionTree<LeafT> ast) {
    ExprTreePrintingVisitor<LeafT> astp(ss);
    boost::apply_visitor( astp, ast.root );
    return ss;
}

std::ostream &
operator<<(std::ostream & ss, const Nil & ast) {
    ss << "(nil)";
    return ss;
}

template<> void
LeafPrintingTraits< typename AST::Leaf >::print_leaf( const typename AST::Leaf & p
                                                    , std::ostream & ss ) {
    ss << "[";
    switch( p.first ) {
        case unset : ss << '0' ; break;
        case eq    : ss << '=' ; break;
        case neq   : ss << '!' ; break;
        case gt    : ss << '>' ; break;
        case gte   : ss << ">="; break;
        case lt    : ss << '<' ; break;
        case lte   : ss << "<="; break;
        default: ss << '?'; break;
    };
    ss << ", \"" << p.second
       << "\"]";
}

template<typename T>
struct LeafPrintingTraits< std::shared_ptr<T> > {
    static void print_leaf(
            const std::shared_ptr<T> & p
          , std::ostream & ss ) {
        const T & obj = *p;
        ss << obj;
    }
};

/**@}*/

template<typename SubjectT> struct Traits;  // fwd

/**\defgroup Common
 * Common utility implementations. */
/**@{*/

namespace _impl {

template< typename LeafT > struct ExpressionEvaluationVisitor;  // fwd

/// Polymorphic base for specialized filters.
template< typename SubjectT
        , typename ResultT>
struct AbstractFilter {

    virtual void dump( std::ostream & ) const = 0;
    virtual ResultT eval( SubjectT ) const = 0;

    friend std::ostream & operator<<( std::ostream & os, const AbstractFilter & me ) {
        me.dump( os );
        return os;
    }
};

/// Common base for parameterized filters capable to parse arbitrary
/// sub-expressions, boilerplate for specialized filters.
template< typename ValueT
        , typename SubjectT
        , typename ResultT >
class ParameterizedFilter : public AbstractFilter< SubjectT
                                                 , ResultT > {
public:
    enum BinOpType { AND_, OR_ };
    typedef ValueT Value;
    template<BinOpType> struct BinaryOperator;
    /// Leaf node -- performs user-defined comparison w.r.t. predicate.
    struct Leaf {
        typedef ValueT Value;
        typedef bool Result;

        /// Keeps ctrd pattern to match. Must be trivially ctrble.
        Value pattern;
        /// Ptr to comparator function
        Result (*comparator)( Value l, Value r );

        Result operator()( const Value & v ) const {
            assert(comparator);
            return comparator( v, pattern );
        }

        friend std::ostream & operator<<( std::ostream & os, const Leaf & l ) {
            return os << "[" << (void *) l.comparator
                      << ", " << l.pattern
                      << "]";
        }
    };
    /// Represents comparison pattern (e.g. >=100, !=0, etc.).
    typedef ExpressionTree<Leaf> DecisionTree;
private:
    DecisionTree dt;
public:
    ParameterizedFilter( DecisionTree dt_ ) : dt(dt_) {}
    /// Evaluates expression.
    ResultT eval( SubjectT s ) const override {
        ExpressionEvaluationVisitor<Leaf> v(compute( s ));
        //return boost::apply_visitor(v, dt.root);
        return v(dt.root);
    }
    /// Must compute value from given subject instance.
    virtual Value compute( SubjectT s ) const = 0;

    void dump(std::ostream & ss) const override {
        ucdslang::ExprTreePrintingVisitor<Leaf> pv( ss );
        boost::apply_visitor( pv, dt.root );
    }
};

}  // namespace _impl

template<typename T>
struct ComparisonTraits {
    static bool  eq( T l, T r ) { return l == r; }
    static bool neq( T l, T r ) { return l != r; }
    static bool  gt( T l, T r ) { return l >  r; }
    static bool gte( T l, T r ) { return l >= r; }
    static bool  lt( T l, T r ) { return l <  r; }
    static bool lte( T l, T r ) { return l <= r; }
    static bool (*cmpPtrs[7])(T, T);
};

template<typename T> bool (*ComparisonTraits<T>::cmpPtrs[7])(T, T) = {
        ComparisonTraits::eq,   ComparisonTraits::neq,
        ComparisonTraits::gt,   ComparisonTraits::gte,
        ComparisonTraits::lt,   ComparisonTraits::lte
    };

namespace helpers {

template< typename T
        , typename CtxT>
static typename Traits<CtxT>::template Parameterized<T>::Leaf
parse_subexpr( Predicate p, const std::string & s ) {
    typename Traits<CtxT>::template Parameterized<T>::Leaf l;
    l.pattern = boost::lexical_cast<T>( s );
    if( ::ucdslang::unset == p ) {
        l.comparator = NULL;
        return l;
    }
    l.comparator = ComparisonTraits<T>::cmpPtrs[ ((int) p) - 1 ];
    return l;
}

}  // namespace helpers
/**@}*/


namespace _impl {
template<typename AbstractFilterT> class VCtrRegistry;  // fwd
}  // namespace _impl

/**\defgroup Filtering Traits
 * Traits scope-struct. */
/**@{*/

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

/**@}*/

/**\defgroup Evaluation
 * Filter condition evaluation routines. */
/**@{*/
namespace _impl {
template< typename LeafT >
struct LeafEvalTraits {
    typedef typename LeafT::Value Value;
    typedef typename LeafT::Result Result;
    static Result eval_leaf( const LeafT & l
                           , const Value & v ) {
        return l( v );
    }
    //static Result eval_and( const Tree<LeafT> & l ) {
    // ...?
    //}
};

/// Condition evaluator recursive visitor struct.
template< typename LeafT >
struct ExpressionEvaluationVisitor : public boost::static_visitor<typename LeafT::Result> {
    typedef LeafT Leaf;
    typedef ExpressionTree<Leaf> Tree;
    typedef typename LeafEvalTraits<Leaf>::Value Value;
    typedef typename LeafEvalTraits<Leaf>::Result Result;

    Value val;

    ExpressionEvaluationVisitor( Value v ) : val(v) {}

    Result operator()( Nil ) const { assert(false); }
    Result operator()( const Leaf & l ) const {
        return LeafEvalTraits<Leaf>::eval_leaf( l, val );
    }
    Result operator()( const typename Tree::template BinaryOperator<AND_> & and_ ) const {
        if( !boost::apply_visitor(*this, and_.l.root) ) return false;
        if( !boost::apply_visitor(*this, and_.r.root) ) return false;
        return true;
    }
    Result operator()( const typename Tree::template BinaryOperator<OR_> & or_ ) const {
        if( boost::apply_visitor(*this, or_.l.root) ) return true;
        if( boost::apply_visitor(*this, or_.r.root) ) return true;
        return false;
    }
    Result operator()( const Tree & dt ) const {
        return boost::apply_visitor( *this, dt.root );
    }
};
}  // namespace _impl
/**@}*/

/**\defgroup Constructing
 * Filter condition constructing routines. */
/**@{*/
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
/**@}*/


/**\defgroup Parsing
 * Expression parsing utilities. */
/**@{*/
namespace parser {

namespace qi = boost::spirit::qi;
namespace ascii = boost::spirit::ascii;
namespace spirit = boost::spirit;
namespace phx = boost::phoenix;

template< typename ItT
        , typename T>
struct CtxFilter : qi::grammar< ItT
                              , typename ucdslang::Traits<T>::Tree
                              , ascii::space_type > {

    CtxFilter( std::ostream * logStr_=nullptr ) : CtxFilter::base_type(filters)
                                                , logStr(logStr_) {
        using qi::char_;
        using ascii::string;
        using qi::lit;
        using qi::_val;
        using qi::lexeme;
        using boost::spirit::ascii::alpha;

        filters =
            filter[_val = qi::_1]
            >> *( ( lit("&&") >> filter )[ _val &= qi::_1 ]
                | ( lit("||") >> filter )[ _val |= qi::_1 ]
                )
            ;

        filter = (label >> char_(':') >> expressions)[_val = filter_vctr(qi::_1, qi::_3)]
            | (char_('(') >> filters >> ')')[_val = qi::_2]
            ;

        expressions = expression[_val = qi::_1]
            >> *( (char_('|') >> expression)[ _val |= qi::_2 ]
                | (char_('&') >> expression)[ _val &= qi::_2 ]
                )
            ;

        expression = term[ _val = qi::_1 ]
            | (char_('(') >> expressions >> ')')[_val = qi::_2]
            ;

        term = (predicate_ >> ctrParameter)[
                    qi::_val = phx::construct<
                            std::pair<ucdslang::Predicate, std::string>
                        >( qi::_1, qi::_2 )
                ]
            ;

        ctrParameter = lexeme[+(char_ - (char_('&') | '|' | '(' | ')'))]  // TODO: support for escaped chars
            ;

        label %= (qi::alpha | qi::char_( "_" ))
                >> *(qi::alnum | qi::char_( "_" ) | qi::char_( "." ))
            ;

        BOOST_SPIRIT_DEBUG_NODE(filters);
        BOOST_SPIRIT_DEBUG_NODE(filter);
        BOOST_SPIRIT_DEBUG_NODE(expression);
        BOOST_SPIRIT_DEBUG_NODE(expressions);
        BOOST_SPIRIT_DEBUG_NODE(term);
        BOOST_SPIRIT_DEBUG_NODE(label);
        BOOST_SPIRIT_DEBUG_NODE(ctrParameter);
    }

    std::ostream * logStr;

    qi::rule< ItT
            , typename ucdslang::Traits<T>::Tree
            , ascii::space_type
            > filter, filters;
    qi::rule< ItT
            , ucdslang::AST()
            , ascii::space_type> expression, expressions;
    qi::rule< ItT
            , std::pair<ucdslang::Predicate, std::string>
            , ascii::space_type > term;

    qi::rule<ItT, std::string(), ascii::space_type> label
                                                  , ctrParameter;

public:
    struct FilterVCtrImpl {
        typedef std::shared_ptr<typename ucdslang::Traits<T>::AbstractFilter> ResHandle;

        template<typename Sig> struct result;

        template<typename This, typename A0, typename A1>
        struct result< This(A0, A1) > {
            typedef ResHandle type;
        };

        ResHandle operator()( std::string name, ucdslang::AST ast ) const {
            return ResHandle(ucdslang::Traits<T>::produce( name, ast, nullptr ));
        }
    };


private:
    struct Predicate_ : qi::symbols<char, ucdslang::Predicate> {
        Predicate_() {
            this->add( "=" , ucdslang::eq )
                     ( "!" , ucdslang::neq )
                     ( ">" , ucdslang::gt )
                     ( ">=", ucdslang::gte )
                     ( "<" , ucdslang::lt )
                     ( "<=", ucdslang::lte )
                     ;
        }
    } predicate_;

    boost::phoenix::function<FilterVCtrImpl> filter_vctr;
};

}  // namespace parser
/**@}*/
}  // namespace ucdslang

# endif  // _H_uCONDDSL_H_

