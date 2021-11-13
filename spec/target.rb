# frozen_string_literal: true

require "yaml"

# A class that represents build target

class Target
  VALID_KEYS = %w[name].freeze

  def initialize(hash)
    raise ArgumentError, "expects Hash, but got `#{hash.class}`" unless hash.is_a?(Hash)
    validate_keys(hash)

    @metadata = hash
  end

  attr_reader :metadata

  def validate_keys(hash)
    hash.each_key do |k|
      raise ArgumentError, "unknown key: `#{k}`. valid keys are: #{VALID_KEYS.join(' ')}" unless VALID_KEYS.include? k
    end
  end

  def name?
    metadata.key?("name")
  end

  def name
    metadata["name"]
  end
end
