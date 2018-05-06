# ifndef _H_UCONDSL_TREE_H_
# define _H_UCONDSL_TREE_H_

# include "uc-dsl-types.hpp"

# include <utility>
# include <boost/variant.hpp>

namespace ucdslang {

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

}  // namespace ucdslang

# endif  // _H_UCONDSL_TREE_H_


