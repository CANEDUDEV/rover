can_kingdom_inc = include_directories('include')

can_kingdom_mayor_src = files(
  'src' / 'mayor.c',
  'src' / 'types.c',
)

can_kingdom_king_src = files(
  'src' / 'king.c',
  'src' / 'types.c',
)

can_kingdom_mayor_dep = declare_dependency(
  include_directories: can_kingdom_inc,
  sources: can_kingdom_mayor_src,
)

can_kingdom_king_dep = declare_dependency(
  include_directories: can_kingdom_inc,
  sources: can_kingdom_king_src,
)

subdir('tests')

can_kingdom_tidy_files = [
  can_kingdom_mayor_src,
  can_kingdom_king_src,
  can_kingdom_postmaster_dummy_src,
  can_kingdom_test_tidy_files,
]
