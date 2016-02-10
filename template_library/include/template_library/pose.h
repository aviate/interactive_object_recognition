#ifndef POSE_H_
#define POSE_H_

#include <boost/preprocessor.hpp>

#define ENUM_WITH_STRING_CASE(r, data, elem)    \
    case elem : return BOOST_PP_STRINGIZE(elem);

#define DEFINE_ENUM_WITH_STRING_CONVERSIONS(name, enumerators)          \
    enum name {                                                         \
        BOOST_PP_SEQ_ENUM(enumerators)                                  \
    };                                                                  \
                                                                        \
    inline const char* name##_AS_STRING(name v)                         \
    {                                                                   \
        switch (v)                                                      \
        {                                                               \
            BOOST_PP_SEQ_FOR_EACH(                                      \
                ENUM_WITH_STRING_CASE,                                  \
                name,                                                   \
                enumerators                                             \
            )                                                           \
            default: return "[Unknown " BOOST_PP_STRINGIZE(name) "]";   \
        }                                                               \
    }

/**
 * Contains pose-related definitions.
 */
namespace Pose {
	/**
	* @brief       Available poses.
	*/
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(     \
		POSE,                                \
		(Z_DOWN)                             \
		(Z_DOWN_SPINE)                       \
		(Z_UP_SPINE)                         \
		(Z_UP)                               \
		(POSE_UNKNOWN)                       \
	)

	/**
	 * @brief      Number of possible poses.
	 */
	const int POSE_COUNT = 4;
}

#endif /* POSE_H_ */
