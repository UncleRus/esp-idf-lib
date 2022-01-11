# frozen_string_literal: true

require "yaml"
require_relative "group"

class GroupList
  # path to `groups.yml`
  def initialize(arg)
    @path = File.expand_path(arg)
  end

  attr_reader :path

  def load_file
    File.read(path)
  end

  def parse
    YAML.safe_load(load_file)
  end

  def metadata
    return @metadata if @metadata

    @metadata = parse
  end

  def all
    metadata.map { |g| Group.new(g) }
  end

  def lookup(name)
    metadata.select { |g| g["name"] == name }.map { |g| Group.new(g) }
  end
end
