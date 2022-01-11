# frozen_string_literal: true

require "yaml"

class PersonList
  # path to `persons.yml`
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
    metadata
  end

  def lookup(name)
    metadata.select { |g| g["name"] == name }
  end
end
