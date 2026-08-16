idf_component_register(
    SRCS "web_control.c"
    INCLUDE_DIRS "."
    REQUIRES esp_http_server log motor
    
    EMBED_FILES "html/index.html"
)