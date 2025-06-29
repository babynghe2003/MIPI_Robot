file(GLOB SOURCES src/*.c)

idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS "include"
		    PRIV_REQUIRES driver)
