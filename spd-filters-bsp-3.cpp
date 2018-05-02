# include <boost/spirit/include/qi.hpp>
# include <boost/phoenix/object/construct.hpp>
# include <boost/phoenix/bind/bind_function.hpp>
# include <boost/phoenix/core/is_nullary.hpp>
# include <boost/bind.hpp>
# include <unordered_map>

//#include <boost/spirit/include/karma.hpp>
//#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
//#include <boost/fusion/include/std_pair.hpp>

# include <iostream>
# include <string>

namespace filtering {

struct AST;
struct nil {};

struct AST {
    typedef boost::variant< nil
        , std::string
        > type;

    AST( ) : expr(nil()) {}

    template<typename Expr>
    AST( const Expr & expr_ ) : expr(expr_) {}

    type expr;
};

}  // namespace filtering

namespace client {

namespace qi = boost::spirit::qi;
namespace ascii = boost::spirit::ascii;
namespace spirit = boost::spirit;
namespace phx = boost::phoenix;

template<typename ItT>
struct CtxFilter : qi::grammar< ItT
                              , filtering::AST()
                              , ascii::space_type > {


    CtxFilter() : CtxFilter::base_type(expression) {
        using qi::char_;
        using qi::_val;

        expression = (char_('_') >> symbolRaw) [ _val = phx::construct<std::string>(qi::_2) ];
        symbolRaw = +(qi::alpha | qi::char_( "_" ));

        BOOST_SPIRIT_DEBUG_NODE(expression);
    }

    qi::rule< ItT
            , filtering::AST()
            , ascii::space_type > expression;

    qi::rule< ItT
            , std::string()
            , ascii::space_type> symbolRaw;
};

}

int
main( int argc, char * const argv[] ) {
    typedef client::CtxFilter<std::string::const_iterator> CtxFilter;
    CtxFilter ctxFilter;
    std::string str;
    while( std::getline(std::cin, str) ) {
        if (str.empty() || str[0] == 'q' || str[0] == 'Q')
            break;

        std::string::const_iterator iter = str.begin();
        std::string::const_iterator end = str.end();
        bool r = phrase_parse(iter, end, ctxFilter, boost::spirit::ascii::space);

        if (r && iter == end) {
            std::cout << "Parsing succeeded: " /*<< ut*/ << std::endl;
        } else {
            std::string rest(iter, end);
            std::cout << "Parsing failed at:\n" << rest << std::endl;
        }
    }
    return 0;
}

