doxyfile = files('Doxyfile')

doxygen = find_program('doxygen', required: false)

doxygen_input_src = '@0@/apps @0@/libs/can-kingdom'.format(
    meson.project_source_root(),
)

doxygen_xml = custom_target(
    'doxygen',
    command: [doxygen, '@INPUT@'],
    input: doxyfile,
    env: {'DOXYGEN_INPUT': doxygen_input_src},
    output: 'xml',
    build_always_stale: true,
)

sphinx = find_program('sphinx-build', required: false)

sphinx_project_arg = '-Dbreathe_projects.@0@=@1@'.format(
    meson.project_name(),
    doxygen_xml.full_path(),
)

sphinx_src = files('conf.py', 'index.rst')

custom_target(
    'docs',
    command: [
        sphinx,
        '-W',
        '-a',
        '-q',
        '-j=auto',
        '-E',
        sphinx_project_arg,
        meson.current_source_dir(),
        meson.current_build_dir() / 'html',
    ],
    input: sphinx_src,
    # depends: [doxygen_xml, download_assets],
    depends: doxygen_xml,
    output: 'html',
    build_always_stale: true,
)
