build_lib_example(
	NAME vlc-example
	SOURCE_FILES
		vlc-example.cc

	LIBRARIES_TO_LINK
		${libvlc}
		${libcore}
		${libnetwork}
		${libinternet}
		${libapplications}
		${libpoint-to-point}
		${libnetanim}
)
