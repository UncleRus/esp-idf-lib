# frozen_string_literal: true

require "yaml"

# A class that represents build target

class Target
  def initialize(hash)
    raise ArgumentError, "expects Hash, but got `#{hash.class}`" unless hash.kind_of?(Hash)
    raise ArgumentError, "name is missing" unless hash.key?("name")

    @name = hash["name"]
  end

  attr_reader :name
end
