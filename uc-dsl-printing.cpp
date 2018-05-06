# include "uc-dsl-printing.tcc"

namespace ucdslang {

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

}  // namespace ucdslang

