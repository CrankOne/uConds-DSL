# ifndef _H_UCONDSL_PRINTING_H_
# define _H_UCONDSL_PRINTING_H_

# include "uc-dsl-tree.tcc"

# include <boost/variant/static_visitor.hpp>

namespace ucdslang {

// Specialize this template to print leaf of certain type.
template<typename LeafT>
struct LeafPrintingTraits {
    static void print_leaf( const LeafT & l, std::ostream & os) { os << "[" << l << "]"; }
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

}  // namespace ucdslang

# endif  // _H_UCONDSL_PRINTING_H_

