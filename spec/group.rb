# frozen_string_literal: true

class Group
  VALID_KEYS = %w[name description].freeze

  def initialize(arg)
    validate_arg(arg)
    validate_keys(arg) if arg.is_a? Hash

    @metadata = if arg.is_a? String
                  { "name" => arg }
                else
                  arg
                end
  end

  def validate_keys(arg)
    arg.each_key do |k|
      raise ArgumentError, "unknown key: `#{k}`. valid keys are: #{VALID_KEYS.join(' ')}" unless VALID_KEYS.include? k
    end
    raise ArgumentError, "a key, `name` is required, but missing" unless arg.key?("name")
  end

  def validate_arg(arg)
    raise ArgumentError, "argument must be String or Hash" unless arg.is_a?(String) || arg.is_a?(Hash)
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

  def to_s
    @metadata["name"]
  end
end
