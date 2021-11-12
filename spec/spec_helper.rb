# frozen_string_literal: true

require "rspec"
require "yaml"

@component_dir = File.expand_path(File.join(File.dirname(__FILE__), "../components"))

class Component
  def initialize(path)
    @path = path
    @name = File.basename(path)
  end

  attr_reader :path, :name

  def to_s
    name
  end

  def metadata
    return @metadata if @metadata

    @metadata = YAML.safe_load(File.read(File.join(path, ".eil.yml")))
  end
end

def components
  directories = Dir.children(Dir.new(@component_dir)).map { |c| File.join(@component_dir, c) }
  directories.select { |d| File.directory?(d) }.map { |path| Component.new(path) }
end
