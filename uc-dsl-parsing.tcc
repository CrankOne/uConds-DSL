# ifndef _H_UCONDSL_PARSING_H_
# define _H_UCONDSL_PARSING_H_

//# define BOOST_RESULT_OF_USE_DECLTYPE
//# define BOOST_SPIRIT_USE_PHOENIX_V3

# include "uc-dsl-traits.tcc"
# include "uc-dsl-evaluation.tcc"
# include "uc-dsl-constructing.tcc"

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

namespace ucdslang {
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
}  // namespace ucdslang

# endif  // _H_UCONDSL_PARSING_H_

