can_kingdom_inc = include_directories('include')

can_kingdom_src = files(
  'src' / 'king.c',
  'src' / 'mayor.c',
  'src' / 'types.c',
)

can_kingdom_lib = library(
  'can-kingdom',
  can_kingdom_src,
  include_directories: can_kingdom_inc,
)

can_kingdom_tidy_files = can_kingdom_src
