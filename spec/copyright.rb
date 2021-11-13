# frozen_string_literal: true

require_relative "person"

class Copyright
  VALID_KEYS = %w[name year]

  def initialize(hash)
    raise ArgumentError, "expect Hash, got `#{hash.class}`" unless hash.is_a?(Hash)
    raise ArgumentError, "missing key `name`" unless hash.key?("name")

    validate_keys(hash)
    @metadata = hash
  end

  attr_reader :metadata

  def validate_keys(hash)
    hash.each_key do |k|
      raise ArgumentError, "unknown key: `#{k}`. valid keys are: #{VALID_KEYS.join(' ')}" unless VALID_KEYS.include? k
    end
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
