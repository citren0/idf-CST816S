
idf_component_register(
    SRCS "cst816s.c"
    REQUIRES driver
    PRIV_REQUIRES i2c-lock
    INCLUDE_DIRS "include"
)