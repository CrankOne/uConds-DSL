# ifndef _H_UCONDSL_COMMON_H_
# define _H_UCONDSL_COMMON_H_

# include "uc-dsl-tree.tcc"
# include "uc-dsl-printing.tcc"

# include <boost/lexical_cast.hpp>

# include <iosfwd>

namespace ucdslang {

template<typename SubjectT> struct Traits;  // fwd

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

}  // namespace ucdslang

# endif  // _H_UCONDSL_COMMON_H_

