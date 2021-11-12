# frozen_string_literal: true

require_relative "copyright"
require_relative "target"
require_relative "person"
require_relative "license"
require_relative "group"

class Component
  # path: path to component root directory
  def initialize(path)
    raise ArgumentError, "path is missing" unless path

    @path = path
    @name = File.basename(path)
    raise ArgumentError, "path `#{path}` does not have basename" if @name.empty?
  end

  attr_reader :path, :name

  def to_s
    name
  end

  def metadata
    return @metadata if @metadata

    @metadata = YAML.safe_load(File.read(File.join(path, ".eil.yml")))
  end

  def name?
    metadata.key?("name")
  end

  def description?
    metadata.key?("description")
  end

  def description
    metadata["description"]
  end

  def groups?
    metadata.key?("groups")
  end

  def groups
    metadata["groups"].map { |g| Group.new(g) }
  end

  def code_owners?
    metadata.key?("code_owners")
  end

  def code_owners
    metadata["code_owners"].map { |p| Person.new(p) }
  end

  def depends?
    metadata.key?("depends")
  end

  def depends
    metadata["depends"]
  end

  def thread_safe?
    metadata.key?("thread_safe")
  end

  def thread_safe
    metadata["thread_safe"]
  end

  def targets?
    metadata.key?("targets")
  end

  def targets
    metadata["targets"].map { |t| Target.new(t) }
  end

  def licenses?
    metadata.key?("licenses")
  end

  def licenses
    metadata["licenses"].map { |l| License.new(l) }
  end

  def copyrights?
    metadata.key?("copyrights")
  end

  def copyrights
    metadata["copyrights"].map { |c| Copyright.new(c) }
  end
end
