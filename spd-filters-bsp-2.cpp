# include <boost/config/warning_disable.hpp>
# include <boost/spirit/include/qi.hpp>
# include <boost/spirit/include/phoenix_operator.hpp>
# include <boost/phoenix/bind/bind_function.hpp>
# include <boost/phoenix/object/construct.hpp>
# include <boost/fusion/include/std_pair.hpp>
# include <boost/bind.hpp>
# include <boost/lexical_cast.hpp>
# include <boost/algorithm/string/trim.hpp>
# include <boost/move/unique_ptr.hpp>

# include <unordered_map>
# include <iostream>
# include <string>

namespace filtering {

// Abstract syntax tree
// ///////////////////

enum Predicate { unset = 0x0
    , eq  = 0x1, neq = 0x2
    , gt  = 0x3, gte = 0x4
    , lt  = 0x5, lte = 0x6
};

enum BinOpType { AND_, OR_ };

struct Nil {};

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

// Specialize this template to print leaf of certain type.
template<typename LeafT>
struct LeafPrintingTraits {
    static void print_leaf( const LeafT & l, std::ostream & os) { os << l; }
};

template< typename LeafT>
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
        ss << " & ";
        boost::apply_visitor( *this, bo.r.root );
        ss << ")";
    }

    void operator()( const typename Tree::template BinaryOperator<OR_> & bo ) {
        ss << "(";
        boost::apply_visitor( *this, bo.l.root );
        ss << " & ";
        boost::apply_visitor( *this, bo.r.root );
        ss << ")";
    }
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

/// AST for interim representation of single-typed values
/// (expressions in "<label> : <expr>" pairs).
typedef ExpressionTree<std::pair<Predicate, std::string> > AST;

std::ostream & operator<<(std::ostream & ss, AST ast) {
    ExprTreePrintingVisitor<typename AST::Leaf> astp(ss);
    boost::apply_visitor( astp, ast.root );
    return ss;
}

// Filtering traits
//////////////////

/// Traits defined for some particular subject. May be specialized.
template<typename SubjectT>
struct Traits {
    typedef SubjectT Subject;
    /// Abstract base for filter objects.
    struct Base { virtual bool check( Subject ) const = 0; };

    typedef std::shared_ptr<Base> NodeRef;

    typedef ExpressionTree< NodeRef > Tree;

    /// Common base for parameterized filters capable to parse arbitrary
    /// sub-expressions.
    template<typename ValueT>
    class Parameterized : public Base {
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
        Parameterized( DecisionTree dt_ ) : dt(dt_) {}
        /// Evaluates expression.
        bool check( Subject s ) const override {
            ExpressionEvaluationVisitor<Leaf> v(compute( s ));
            //return boost::apply_visitor(v, dt.root);
            return v(dt.root);
        }
        /// Must compute value from given subject instance.
        virtual Value compute( Subject s ) const = 0;
    };

    /// Filter construction callback.
    typedef typename Traits::Base * (*FilterConstructor)( const AST & );

    /// Factory method for ctring the filters.
    static Base * produce( const std::string
                         , const AST &
                         , std::ostream * logStream=nullptr );

    /// Returns named filter ctr or NULL.
    static FilterConstructor constructor( const std::string );

    /// Helper method performing AST treatment during ctrion of new filter instance.
    static Base * construct( FilterConstructor, const AST & );

    /// Registers new ctr making it available within the parsing routines.
    static void register_constructor( const char *, FilterConstructor );
private:
    /// Known constructors.
    static std::unordered_map<std::string, FilterConstructor> * _constructors;
};  // struct Traits

template<typename T>
std::unordered_map<std::string, typename Traits<T>::FilterConstructor>
    * Traits<T>::_constructors = nullptr;

template<typename T> void
Traits<T>::register_constructor( const char * name
                               , typename Traits<T>::FilterConstructor ctr ) {
    if( !_constructors ) {
        _constructors = new std::unordered_map<std::string, typename Traits<T>::FilterConstructor>;
    }
    auto rc = _constructors->emplace( name, ctr );
    if( ! rc.second ) {
        throw std::runtime_error( "Repeatative insertion of filter constructor"
                " \"" + std::string(name) + "\".");
    }
}

template<typename T>
typename Traits<T>::FilterConstructor
Traits<T>::constructor( const std::string name ) {
    assert(_constructors);
    auto it = _constructors->find(name);
    if( _constructors->end() == it ) {
        return nullptr;
    }
    return it->second;
}

template<typename T>
typename Traits<T>::Base *
Traits<T>::produce( const std::string name
                  , const AST & ast
                  , std::ostream * ssPtr ) {
    assert(_constructors);
    auto ctr = Traits<T>::constructor(name);
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
    return Traits<T>::construct(ctr, ast);
}

// Comparison traits
// ////////////////

template<typename T>
struct ComparisonTraits {
    static bool  eq( T l, T r ) { return l == r; }
    static bool neq( T l, T r ) { return l != r; }
    static bool  gt( T l, T r ) { return l >  r; }
    static bool gte( T l, T r ) { return l >= r; }
    static bool  lt( T l, T r ) { return l <  r; }
    static bool lte( T l, T r ) { return l <= r; }
    static bool (*cmpPtrs[7])(T, T);
    template<typename CtxT> static typename Traits<CtxT>::template Parameterized<T>::Leaf
    parse( Predicate p, const std::string & s ) {
        typename Traits<CtxT>::template Parameterized<T>::Leaf l;
        l.pattern = boost::lexical_cast<T>( s );
        if( ::filtering::unset == p ) {
            l.comparator = NULL;
            return l;
        }
        l.comparator = cmpPtrs[ ((int) p) - 1 ];
        return l;
    }
};

template<typename T> bool (*ComparisonTraits<T>::cmpPtrs[7])(T, T) = {
        ComparisonTraits::eq,   ComparisonTraits::neq,
        ComparisonTraits::gt,   ComparisonTraits::gte,
        ComparisonTraits::lt,   ComparisonTraits::lte
    };

namespace aux {
/// Use this visitor to construct expressions within user-side constructors.
template< typename T
        , typename ValueT
        , typename ValueParserT>
class ConstructingVisitor : public boost::static_visitor<typename Traits<T>::Base *> {
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
}  // namespace aux

template<typename T>
typename Traits<T>::Base *
Traits<T>::construct( FilterConstructor ctr, const AST & ast ) {
    assert( ctr );
    return ctr(ast);
}

}  // namespace filtering

// Parsing routines
// ///////////////

namespace client {

namespace qi = boost::spirit::qi;
namespace ascii = boost::spirit::ascii;
namespace spirit = boost::spirit;
namespace phx = boost::phoenix;

template< typename ItT
        , typename T>
struct CtxFilter : qi::grammar< ItT
                              , typename filtering::Traits<T>::Tree
                              , ascii::space_type > {

    CtxFilter( std::ostream * logStr_=nullptr ) : CtxFilter::base_type(filters)
                                                , logStr(logStr_) {
        using qi::char_;
        using ascii::string;
        using qi::lit;
        using qi::_val;
        using qi::lexeme;
        using boost::spirit::ascii::alpha;
        typedef filtering::Traits<T> FT;

        filters =
            filter[_val = qi::_1]
            >> -( ( lit("&&") >> filter )[ _val |= qi::_1 ]
                | ( lit("||") >> filter )[ _val |= qi::_1 ]
                )
            ;

        filter = (label >> char_(':') >> expressions)[phx::bind(&FT::produce, qi::_1, qi::_3, logStr)]
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
                            std::pair<filtering::Predicate, std::string>
                        >( qi::_1, qi::_2 )
                ]
            ;

        ctrParameter = lexeme[+(char_ - (char_('&') | '|' | '(' | ')'))]  // TODO: support for escaped chars
            ;

        label %= (qi::alpha | qi::char_( "_" ))
                >> *(qi::alnum | qi::char_( "_" ))
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
            , typename filtering::Traits<T>::Tree
            , ascii::space_type
            > filters;
    qi::rule< ItT
            , typename filtering::Traits<T>::NodeRef
            , ascii::space_type
            > filter;
    qi::rule< ItT
            , filtering::AST()
            , ascii::space_type> expression, expressions;
    qi::rule< ItT
            , std::pair<filtering::Predicate, std::string>
            , ascii::space_type > term;

    qi::rule<ItT, std::string(), ascii::space_type> label
                                                  , ctrParameter;
private:
    struct Predicate_ : qi::symbols<char, filtering::Predicate> {
        Predicate_() {
            this->add( "=" , filtering::eq )
                     ( "!" , filtering::neq )
                     ( ">" , filtering::gt )
                     ( ">=", filtering::gte )
                     ( "<" , filtering::lt )
                     ( "<=", filtering::lte )
                     ;
        }
    } predicate_;
};

}  // namespace client

namespace filtering {
template<> void
LeafPrintingTraits< typename AST::Leaf >::print_leaf( const typename AST::Leaf & p
                                                    , std::ostream & ss ) {
    ss << "(";
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
       << "\")";
}

}  // namespace filtering

// ////////////////////// ////////////////////// //////////////////////// /////

// Define SomeContext mock ctx struct and couple filters for it.
struct SomeContext { int a, b; };

/// Filter for mock context's "a" attribute
class AComparator : public filtering::Traits<SomeContext>::Parameterized<int> {
public:
    /// Own filtering traits type alias.
    typedef filtering::Traits<SomeContext> FT;
    typedef typename FT::Parameterized<int> Parent;
    /// (interface implem) Acquires "a" value from SomeContext.
    int compute( SomeContext ctx ) const override { return ctx.a; }
    /// Virtual ctr function.
    static typename FT::Base * vctr( const filtering::AST & ast );
private:
    /// Own ctr, available to vctr only.
    AComparator( DecisionTree dt ) : Parent( dt ) {}
};

typename AComparator::FT::Base *
AComparator::vctr( const filtering::AST & ast ) {
    filtering::aux::ConstructingVisitor< SomeContext
        , int
        , decltype(filtering::ComparisonTraits<int>::parse<SomeContext>) *>
        cv(filtering::ComparisonTraits<int>::parse<SomeContext>);
    auto dt = boost::apply_visitor( cv, ast.root );
    {
        filtering::ExprTreePrintingVisitor<
                typename filtering::Traits<SomeContext>::template Parameterized<int>::Leaf  // Parent::Leaf
                >
            pv( std::cout );  // XXX
        boost::apply_visitor( pv, dt );  // XXX
        std::cout << std::endl;
    }
    return new AComparator( dt );
}

/// Filter for mock context's "b" attribute.
class BComparator : public filtering::Traits<SomeContext>::Parameterized<int> {
public:
    /// Own filtering traits type alias.
    typedef filtering::Traits<SomeContext> FT;
    typedef typename FT::Parameterized<int> Parent;
    /// (interface implem) Acquires "a" value from SomeContext.
    int compute( SomeContext ctx ) const override { return ctx.b; }
    /// Virtual ctr function.
    static typename FT::Base * vctr( const filtering::AST & ast );
private:
    /// Own ctr, available to vctr only.
    BComparator( DecisionTree dt ) : Parent( dt ) {}
};

typename BComparator::FT::Base *
BComparator::vctr( const filtering::AST & ast ) {
    filtering::aux::ConstructingVisitor< SomeContext
        , int
        , decltype(filtering::ComparisonTraits<int>::parse<SomeContext>) *>
        cv(filtering::ComparisonTraits<int>::parse<SomeContext>);
    auto dt = boost::apply_visitor( cv, ast.root );
    return new BComparator( dt );
}

int
main( int argc, char * const argv[] ) {
    filtering::Traits<SomeContext>::register_constructor("a", AComparator::vctr);
    filtering::Traits<SomeContext>::register_constructor("b", BComparator::vctr);

    std::cout << "Type an expression or [q or Q] to quit:\n";

    client::CtxFilter< std::string::const_iterator
                     , SomeContext> ctxFilter( &std::cout );

    std::string str;
    while (std::getline(std::cin, str)) {
        if (str.empty() || str[0] == 'q' || str[0] == 'Q')
            break;

        std::string::const_iterator iter = str.begin();
        std::string::const_iterator end = str.end();
        bool r = phrase_parse(iter, end, ctxFilter, boost::spirit::ascii::space);

        if( r && end == iter ) {
            std::cout << "ok." << std::endl;
        } else {
            std::string rest(iter, end);
            std::cerr << "Failed. Stopped at: \": "
                      << rest
                      << "\"" << std::endl;
        }
    }
    std::cout << "Done." << std::endl;
    return 0;
}

