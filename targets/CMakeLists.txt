add_subdirectory(ttsim)
add_subdirectory(ttsimship)

doxygen_add_docs(doxygen ALL)

# enable_testing needed on all levels to allow correct test discovery form project root
enable_testing()