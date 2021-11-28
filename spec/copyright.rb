# frozen_string_literal: true

require_relative "person"

class Copyright
  VALID_KEYS = %w[
    author
    name
    year
  ].freeze

  def initialize(hash)
    raise ArgumentError, "expect Hash, got `#{hash.class}`" unless hash.is_a?(Hash)

    validate_keys(hash)
    @metadata = hash
  end

  attr_reader :metadata

  def validate_keys(hash)
    hash.each_key do |k|
      raise ArgumentError, "unknown key: `#{k}`. valid keys are: #{VALID_KEYS.join(' ')}" unless VALID_KEYS.include? k
    end
  end

  def author?
    metadata.key?("author")
  end

  def author
    Person.new(metadata["author"])
  end

  def name
    Person.new("name" => metadata["name"])
  end

  def name?
    metadata.key?("name")
  end

  def year
    metadata["year"]
  end

  def year?
    metadata.key?("year")
  end
end
