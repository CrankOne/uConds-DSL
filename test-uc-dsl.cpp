# include "uc-dsl.tcc"

// Define SomeContext mock ctx struct and couple filters for it.
struct SomeContext { int a, b; };

/// Filter for mock context's "a" attribute
class AComparator : public ucdslang::Traits<SomeContext>::Parameterized<int> {
public:
    /// Own ucdslang traits type alias.
    typedef ucdslang::Traits<SomeContext> FT;
    typedef typename FT::Parameterized<int> Parent;
    /// (interface implem) Acquires "a" value from SomeContext.
    int compute( SomeContext ctx ) const override { return ctx.a; }
    /// Virtual ctr function.
    static typename FT::AbstractFilter * vctr( const ucdslang::AST & ast );
private:
    /// Own ctr, available to vctr only.
    AComparator( DecisionTree dt ) : Parent( dt ) {}
};

typename AComparator::FT::AbstractFilter *
AComparator::vctr( const ucdslang::AST & ast ) {
    ucdslang::helpers::ConstructingVisitor< SomeContext
        , int
        , decltype(ucdslang::helpers::parse_subexpr<int, SomeContext>) *>
        cv(ucdslang::helpers::parse_subexpr<int, SomeContext>);
    auto dt = boost::apply_visitor( cv, ast.root );
    return new AComparator( dt );
}

/// Filter for mock context's "b" attribute.
class BComparator : public ucdslang::Traits<SomeContext>::Parameterized<int> {
public:
    /// Own ucdslang traits type alias.
    typedef ucdslang::Traits<SomeContext> FT;
    typedef typename FT::Parameterized<int> Parent;
    /// (interface implem) Acquires "a" value from SomeContext.
    int compute( SomeContext ctx ) const override { return ctx.b; }
    /// Virtual ctr function.
    static typename FT::AbstractFilter * vctr( const ucdslang::AST & ast );
private:
    /// Own ctr, available to vctr only.
    BComparator( DecisionTree dt ) : Parent( dt ) {}
};

typename BComparator::FT::AbstractFilter *
BComparator::vctr( const ucdslang::AST & ast ) {
    ucdslang::helpers::ConstructingVisitor< SomeContext
        , int
        , decltype(ucdslang::helpers::parse_subexpr<int, SomeContext>) *>
        cv(ucdslang::helpers::parse_subexpr<int, SomeContext>);
    auto dt = boost::apply_visitor( cv, ast.root );
    return new BComparator( dt );
}

SomeContext testingSamples[] = {
        {   0,   0 },
        {   1,   0 },
        { 100,   0 },
        {   0,   1 },
        {   0, 100 },
        {   1,   1 },
        {   1, 100 },
        { 100,   1 },
        { 100, 100 },
    };

int
main( int argc, char * const argv[] ) {
    {
        typedef ucdslang::Traits<SomeContext> FT;
        typedef ucdslang::_impl::VCtrRegistry<typename FT::AbstractFilter> Registry;
        Registry::register_constructor("a", AComparator::vctr);
        Registry::register_constructor("b", BComparator::vctr);
    }

    std::cout << "Type an expression or [q or Q] to quit:\n";

    ucdslang::parser::CtxFilter< std::string::const_iterator
                               , SomeContext > ctxFilter( &std::cout );
    ucdslang::Traits<SomeContext>::Tree filter;

    std::string str;
    while (std::getline(std::cin, str)) {
        if( str.empty() || str[0] == 'q' || str[0] == 'Q' ) {
            break;
        }

        std::string::const_iterator iter = str.begin();
        std::string::const_iterator end = str.end();
        bool r = phrase_parse(iter, end
                             , ctxFilter
                             , boost::spirit::ascii::space
                             , filter );

        if( r && end == iter ) {
            std::cout << "ok: " << filter << std::endl;
            for( size_t i = 0
               ; i < sizeof(testingSamples)/sizeof(SomeContext)
               ; ++i ) {
                std::cout << "samle #" << i << ": ";
                //if( ucdslang::EvaluationTraits<SomeContext>::eval(filter, testingSamples[i]) ) {
                //    std::cout << "passed";
                //} else {
                //    std::cout << "denied";
                //}
                std::cout << std::endl;
            }
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

