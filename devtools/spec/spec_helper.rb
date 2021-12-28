# frozen_string_literal: true

require "rspec"
require "yaml"
require_relative "metadata"

@component_dir = File.expand_path(File.join(File.dirname(__FILE__), "../../components"))

def metadata_array
  directories = Dir.children(Dir.new(@component_dir)).map { |c| File.join(@component_dir, c) }
  directories = directories.select { |d| File.directory?(d) }
  directories.map { |path| Metadata.new(path) }
end
