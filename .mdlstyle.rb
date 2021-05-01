# frozen_string_literal: true

# our markdown style

all
rule "MD003", style: :atx
rule "MD007", indent: 4
rule "MD013", code_blocks: false, tables: false
rule "MD024", allow_different_nesting: true
rule "MD029", style: :ordered
