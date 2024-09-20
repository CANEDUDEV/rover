circuits_inc = include_directories('include')

circuits_src = files('src' / 'lt6106.c', 'src' / 'voltage-divider.c')

circuits_dep = declare_dependency(
    include_directories: circuits_inc,
    sources: circuits_src,
)

circuits_tidy_files = circuits_src
