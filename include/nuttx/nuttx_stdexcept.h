#ifndef __NUTTX_NUTTX_STDEXCEPT__
#define __NUTTX_NUTTX_STDEXCEPT__


#ifndef CONFIG_LIBBACKTRACE
#include <stdexecpt>
# define NUTTX_TRY try
# define NUTTX_CATCH(x) catch (x)
# define NUTTX_THROW(x) throw x
#else
# define NUTTX_TRY if (true)
# define NUTTX_CATCH(x) if (false)
# define NUTTX_THROW(x) assert(false)
#endif

#endif /* __NUTTX_NUTTX_STDEXCEPT__ */
