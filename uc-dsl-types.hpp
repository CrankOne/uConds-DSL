# ifndef _H_UCONDSL_TYPES_H_
# define _H_UCONDSL_TYPES_H_

# include <iosfwd>

namespace ucdslang {

enum Predicate { unset = 0x0
    , eq  = 0x1, neq = 0x2
    , gt  = 0x3, gte = 0x4
    , lt  = 0x5, lte = 0x6
};

enum BinOpType { AND_, OR_ };

struct Nil {};

std::ostream & operator<<(std::ostream & ss, const Nil & ast);

}  // namespace ucdslang

# endif  // _H_UCONDSL_TYPES_H_

