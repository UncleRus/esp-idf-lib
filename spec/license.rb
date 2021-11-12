# frozen_string_literal: true

class License
  def initialize(hash)
    raise ArgumentError, "missing key `name`" unless hash.key?("name")

    @metadata = hash
  end

  def name
    @metadata["name"]
  end
end
