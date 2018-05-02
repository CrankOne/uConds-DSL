# uConds-DSL

This Domain-Specific Language (DSL) tries to add the pattern matching to the
C++ by means of the hierarchical data structures assembled on run-time.

## Rationale

For the data analysis in C++ it is quite common case when one has to dispatch
the elements of some sequence to the few consuming recepients. The dispatching
itself has to obey of some quite volatile criteria.

    :::C++
    const double & sA = sample.attributeA
               , & sB = sample.attributeB
               ;
    if( sA > 0 && sA < 10 ) {
        consumer1.put(sample);
    } else if ( sA > 10 && sA < 20 ) {
        consumer2.put(sample);
    } else {
        if( sB ) {
            consumer3.put(sample);
        } else {
            consumer4.put(sample);
        }
    }

Most convenient expressive meaning for that case would involve a mechanism
similar to _pattern matching_ present in functional languages.

    TODO: example here of equivalent code on R/Haskell/Rust/OCaml/CLisp/whatever

We can not, unfortunately, rely on any FL this time since including them into
the technological stack of modern science in small collaborations will cost a
lot.

The possible solution would be to mimic the pattern matching mean using
modern C++11+ features, but it will tie the conditions formulation to compile
time while it is often necessary to eval them at the runtime.

Another approach is to implement a small embeddable language by means of some
common popular specialized solution (e.g. `YACC/LEXX` or `boost::spirit`)
introducing fully-dynamic (yet apparently not the most efficient) decision tree
structs into the C++ code.

## Grammar

TODO

## Examples

Filter events by energy, greater than 100 MeVs:

    energy:>100MeV

Choose only the primary \mu+/\mu- particles (Drell-Yann pair):

    partileType:mu-'|mu+'

Join above conditions together with logical "and":

    energy:>100MeV && partileType:mu-'|mu+'

...


