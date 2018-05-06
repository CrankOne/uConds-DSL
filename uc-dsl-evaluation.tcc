# ifndef _H_UCONDSL_EVALUATION_H_
# define _H_UCONDSL_EVALUATION_H_

# include "uc-dsl-tree.tcc"

# include <boost/variant/static_visitor.hpp>

namespace ucdslang {
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
}  // namespace ucdslang

# endif

