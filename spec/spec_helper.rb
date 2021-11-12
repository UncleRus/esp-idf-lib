# frozen_string_literal: true

require "rspec"
require "yaml"
require_relative "component"

@component_dir = File.expand_path(File.join(File.dirname(__FILE__), "../components"))

def components
  directories = Dir.children(Dir.new(@component_dir)).map { |c| File.join(@component_dir, c) }
  directories.select { |d| File.directory?(d) }.map { |path| Component.new(path) }
end
