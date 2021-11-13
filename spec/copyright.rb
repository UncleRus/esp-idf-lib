# frozen_string_literal: true

require_relative "person"

class Copyright
  def initialize(hash)
    raise ArgumentError, "expect Hash, got `#{hash.class}`" unless hash.is_a?(Hash)
    raise ArgumentError, "missing key `name`" unless hash.key?("name")

    @metadata = hash
  end

  attr_reader :metadata

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
