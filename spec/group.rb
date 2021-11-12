# frozen_string_literal: true

class Group

  VALID_KEYS = %w[name description].freeze

  def initialize(hash)
    raise ArgumentError, "group must be a hash, but got #{hash.class}" unless hash.kind_of? Hash
    raise ArgumentError, "missing key `name`" unless hash.key?("name")

    @metadata = hash
  end


  def validate_keys(hash)
    hash.each_key do |k|
      raise ArgumentError, "unknown key: `#{k}`. valid keys are: #{VALID_KEYS.join(' ')}"
    end
  end
  def name?
    @metadata.key?("name")
  end

  def name
    @metadata["name"]
  end

  def description?
    @metadata.key?("description")
  end

  def description
    @metadata["description"]
  end
end
