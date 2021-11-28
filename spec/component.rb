# frozen_string_literal: true

require_relative "copyright"
require_relative "target"
require_relative "person"
require_relative "license"
require_relative "group"

class Component
  # Component represents a component in metadata. it is not a `component` in
  # esp-idf. a metadata may contain multiple Components. usually, an esp-idf
  # component has one `component` and its metadata usually contains one
  # Component. but a project, such as `esp-idf-lib` repository, may have
  # multiple `components`.

  # a list of valid keys. some values of keys are simply String or Integer
  # object. others are objects, or resources, defined in the specification.
  VALID_KEYS = %w[
    name
    description
    group
    groups
    code_owners
    depends
    thread_safe
    targets
    licenses
    copyrights
  ].freeze

  def initialize(hash)
    validate_keys(hash)
    @metadata = hash
    @name = name
  end

  attr_reader :metadata

  def validate_keys(hash)
    # validate basic constraints only. the classes are for tests in spec
    # files, providing readable tests and results. the actual specification is
    # in the spec files, not in classes. rspec is more readable and maintainable
    # than ruby code.
    #
    # maybe, if these classes are found to be useful, create a gem of the
    # classes and specification tests in the gem. until then, keep the classes
    # simple so that others can maintain the specification.
    raise ArgumentError, "missing name" unless hash.key?("name")
    raise ArgumentError, "empty name" if hash["name"].empty?

    hash.each_key do |k|
      raise ArgumentError, "unknown key: `#{k}`" unless VALID_KEYS.include?(k)
    end
  end

  def to_s
    metadata["name"]
  end

  # special keys that return instances of classes.
  def group
    Group.new(metadata["group"])
  end

  def groups
    metadata["groups"].map { |g| Group.new(g) }
  end

  def code_owners
    metadata["code_owners"].map { |p| Person.new(p) }
  end

  def targets
    metadata["targets"].map { |t| Target.new(t) }
  end

  def licenses
    metadata["licenses"].map { |l| License.new(l) }
  end

  def copyrights
    metadata["copyrights"].map { |c| Copyright.new(c) }
  end

  def valid_key_with_question?(name)
    name.to_s.end_with?("?") && VALID_KEYS.include?(name.to_s.chop)
  end

  def valid_key?(name)
    VALID_KEYS.include?(name.to_s)
  end

  def description
    # if description contains newline, remove it
    metadata["description"].split("\n").join(" ")
  end

  def method_missing(name, *args, &block)
    # name?, description?, etc
    return metadata.key?(name.to_s.chop) if valid_key_with_question?(name)
    # name, etc
    return metadata[name.to_s] if valid_key?(name)

    super
  end

  def respond_to_missing?(name, include_private = false)
    # when name is not something we don't know, do `super`, i.e. raising
    # unknown methods error.
    super unless valid_key_with_question?(name) || valid_key?(name)
  end

  def group_of?(arg)
    group_name = arg.respond_to?(:name) ? arg.name : arg
    group.name == group_name || groups.map(&:name).include?(group_name)
  end
end
